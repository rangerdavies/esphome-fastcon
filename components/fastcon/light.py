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
CONF_DYNAMIC_GROUP = "dynamic_group"
CONF_GROUP_NAME = "group_name"

# Only the fastcon.set_members action key. Membership is no longer declarable on the
# entity itself - it comes from Home Assistant at runtime.
CONF_MEMBERS = "members"

MAX_LIGHT_ID = 255

fastcon_ns = cg.esphome_ns.namespace("fastcon")
FastconLight = fastcon_ns.class_("FastconLight", light.LightOutput, cg.Component)

MODE_SINGLE = fastcon_ns.FASTCON_SINGLE
MODE_GROUP = fastcon_ns.FASTCON_GROUP
MODE_SHARED_GROUP = fastcon_ns.FASTCON_SHARED_GROUP


def _validate_addressing(config):
    modes = [k for k in (CONF_LIGHT_ID, CONF_GROUP_ID, CONF_DYNAMIC_GROUP) if k in config]

    if len(modes) > 1:
        raise cv.Invalid(
            f"Give exactly one of '{CONF_LIGHT_ID}', '{CONF_GROUP_ID}' or "
            f"'{CONF_DYNAMIC_GROUP}', not {', '.join(repr(m) for m in modes)}"
        )
    if not modes:
        raise cv.Invalid(
            f"Give one of '{CONF_LIGHT_ID}' (a single light), '{CONF_DYNAMIC_GROUP}: true' "
            f"(a group whose members come from the fastcon.set_members action), or "
            f"'{CONF_GROUP_ID}' (a group id that already exists in the mesh)"
        )
    if CONF_MEMBERS in config:
        raise cv.Invalid(
            f"'{CONF_MEMBERS}' is no longer set on the light. Declare the entity with "
            f"'{CONF_DYNAMIC_GROUP}: true' and set its membership at runtime with the "
            f"fastcon.set_members action."
        )
    return config


CONFIG_SCHEMA = cv.All(
    light.BRIGHTNESS_ONLY_LIGHT_SCHEMA
    .extend(
        {
            # Names the platform output. fastcon.set_members takes the same value under
            # the same key, so a group is referred to by one name everywhere.
            cv.GenerateID(CONF_GROUP_NAME): cv.declare_id(FastconLight),
            cv.Optional(CONF_OUTPUT_ID): cv.invalid(
                f"'output_id' is now '{CONF_GROUP_NAME}' - the same name the "
                f"fastcon.set_members action uses to refer to this entity"
            ),
            cv.Optional(CONF_LIGHT_ID): cv.int_range(min=1, max=MAX_LIGHT_ID),
            cv.Optional(CONF_GROUP_ID): cv.int_range(min=0, max=255),
            cv.Optional(CONF_DYNAMIC_GROUP): cv.boolean,
            # Accepted only so the removal can be reported clearly instead of as an
            # "unknown option" error; _validate_addressing() always rejects it.
            cv.Optional(CONF_MEMBERS): cv.valid,
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
            cv.Required(CONF_GROUP_NAME): cv.use_id(FastconLight),
            cv.Required(CONF_MEMBERS): cv.templatable(
                cv.ensure_list(cv.int_range(min=1, max=MAX_LIGHT_ID))
            ),
            # Accepted only so the rename can be reported clearly rather than as an
            # "extra keys not allowed" error.
            cv.Optional(CONF_ID): cv.invalid(
                f"'id' is now '{CONF_GROUP_NAME}' on fastcon.set_members - it names the "
                f"group entity whose membership is being set"
            ),
        }
    ),
    # play() packs the mask and re-applies state inline, so play_next_() always runs
    # before play_complex() returns. Nothing is deferred to a timer, callback or loop().
    synchronous=True,
)
async def set_members_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_GROUP_NAME])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    members = await cg.templatable(
        config[CONF_MEMBERS], args, cg.std_vector.template(cg.int32)
    )
    cg.add(var.set_members(members))
    return var


async def to_code(config):
    if CONF_LIGHT_ID in config:
        mode, addr = MODE_SINGLE, config[CONF_LIGHT_ID]
    elif CONF_GROUP_ID in config:
        mode, addr = MODE_GROUP, config[CONF_GROUP_ID]
    else:
        # A dynamic group: it borrows the controller's shared slot and starts with no
        # membership until fastcon.set_members gives it one.
        mode, addr = MODE_SHARED_GROUP, 0

    var = cg.new_Pvariable(config[CONF_GROUP_NAME], addr)
    cg.add(var.set_mode(mode))

    await cg.register_component(var, config)
    await light.register_light(var, config)

    if config.get(CONF_COLOR_INTERLOCK):
        cg.add(var.set_color_interlock(True))

    controller = await cg.get_variable(config[CONF_CONTROLLER_ID])
    cg.add(var.set_controller(controller))

    # Hand the entity its own LightState and register it with the controller, so the
    # passive sniffer can publish an overheard command onto it. Without this the output
    # has no way back to the entity until write_state() first hands it one.
    cg.add(var.set_light_state(await cg.get_variable(config[CONF_ID])))
    cg.add(controller.register_light(var))

    if config.get(CONF_SUPPORTS_CWWW):
        cg.add(var.set_supports_cwww(True))
