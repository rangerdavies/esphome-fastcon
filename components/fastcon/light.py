"""Light platform for Fastcon BLE lights."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import light
from esphome.const import CONF_COLOR_INTERLOCK, CONF_ID, CONF_LIGHT_ID, CONF_OUTPUT_ID
from .fastcon_controller import FastconController

# New config key to toggle RGBCW capability per-entity
CONF_SUPPORTS_CWWW = "supports_cwww"

DEPENDENCIES = ["esp32_ble"]
AUTO_LOAD = ["light"]

CONF_CONTROLLER_ID = "controller_id"
CONF_GROUP_ID = "group_id"
CONF_MEMBERS = "members"

MAX_LIGHT_ID = 255

fastcon_ns = cg.esphome_ns.namespace("fastcon")
FastconLight = fastcon_ns.class_("FastconLight", light.LightOutput, cg.Component)

MODE_SINGLE = fastcon_ns.FASTCON_SINGLE
MODE_GROUP = fastcon_ns.FASTCON_GROUP
MODE_SHARED_GROUP = fastcon_ns.FASTCON_SHARED_GROUP


def _validate_addressing(config):
    has_light = CONF_LIGHT_ID in config
    has_group = CONF_GROUP_ID in config
    has_members = CONF_MEMBERS in config

    if has_light and (has_group or has_members):
        raise cv.Invalid(
            f"'{CONF_LIGHT_ID}' addresses a single light, so it cannot be combined "
            f"with '{CONF_GROUP_ID}' or '{CONF_MEMBERS}'"
        )
    if not (has_light or has_group or has_members):
        raise cv.Invalid(
            f"Give one of '{CONF_LIGHT_ID}' (a single light), '{CONF_MEMBERS}' "
            f"(a named group defined on demand), or '{CONF_GROUP_ID}' "
            f"(a group id that already exists in the mesh)"
        )
    if has_members and config.get(CONF_GROUP_ID) == 0:
        raise cv.Invalid(
            f"'{CONF_GROUP_ID}: 0' is the all-lights group built into the firmware. "
            f"Its membership cannot be set, so remove '{CONF_MEMBERS}'."
        )
    return config


CONFIG_SCHEMA = cv.All(
    light.BRIGHTNESS_ONLY_LIGHT_SCHEMA
    .extend(
        {
            cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(FastconLight),
            cv.Optional(CONF_LIGHT_ID): cv.int_range(min=1, max=MAX_LIGHT_ID),
            cv.Optional(CONF_GROUP_ID): cv.int_range(min=0, max=255),
            cv.Optional(CONF_MEMBERS): cv.All(
                cv.ensure_list(cv.int_range(min=1, max=MAX_LIGHT_ID)),
                cv.Length(min=1),
            ),
            cv.Optional(CONF_CONTROLLER_ID, default="fastcon_controller"): cv.use_id(FastconController),
            cv.Optional(CONF_SUPPORTS_CWWW, default=False): cv.boolean,
            cv.Optional(CONF_COLOR_INTERLOCK, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _validate_addressing,
)


SetMembersAction = fastcon_ns.class_("SetMembersAction", automation.Action)


@automation.register_action(
    "fastcon.set_members",
    SetMembersAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(FastconLight),
            cv.Required(CONF_MEMBERS): cv.templatable(
                cv.ensure_list(cv.int_range(min=1, max=MAX_LIGHT_ID))
            ),
        }
    ),
)
async def set_members_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    members = await cg.templatable(
        config[CONF_MEMBERS], args, cg.std_vector.template(cg.int32)
    )
    cg.add(var.set_members(members))
    return var


def build_membership_mask(members):
    """Pack light ids into the little-endian bitmask the mesh expects.

    Bit N of byte K addresses light_id 8*K + N + 1, so the mask grows a byte for
    every eight lights. Only the single-byte form has been confirmed against
    hardware; wider masks follow the same rule by extension.
    """
    width = (max(members) + 7) // 8
    mask = [0] * width
    for member in members:
        mask[(member - 1) // 8] |= 1 << ((member - 1) % 8)
    return mask


async def to_code(config):
    if CONF_LIGHT_ID in config:
        mode, addr = MODE_SINGLE, config[CONF_LIGHT_ID]
    elif CONF_GROUP_ID in config:
        mode, addr = MODE_GROUP, config[CONF_GROUP_ID]
    else:
        # A named group with no id of its own: it borrows the controller's shared slot.
        mode, addr = MODE_SHARED_GROUP, 0

    var = cg.new_Pvariable(config[CONF_OUTPUT_ID], addr)
    cg.add(var.set_mode(mode))

    await cg.register_component(var, config)
    await light.register_light(var, config)

    if config.get(CONF_COLOR_INTERLOCK):
        cg.add(var.set_color_interlock(True))

    controller = await cg.get_variable(config[CONF_CONTROLLER_ID])
    cg.add(var.set_controller(controller))

    if config.get(CONF_SUPPORTS_CWWW):
        cg.add(var.set_supports_cwww(True))

    if CONF_MEMBERS in config:
        mask = build_membership_mask(config[CONF_MEMBERS])
        cg.add(
            var.set_members(
                cg.RawExpression("{" + ", ".join(f"0x{b:02x}" for b in mask) + "}")
            )
        )
