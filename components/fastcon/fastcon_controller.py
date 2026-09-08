import esphome.codegen as cg
from esphome import automation
from esphome.components import esp32_ble_tracker
from esphome.components import time as time_
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TIME_ID
from esphome.core import HexInt

DEPENDENCIES = ["esp32_ble", "esp32_ble_tracker"]

CONF_MESH_KEY = "mesh_key"
CONF_ADV_INTERVAL_MIN = "adv_interval_min"
CONF_ADV_INTERVAL_MAX = "adv_interval_max"
CONF_ADV_DURATION = "adv_duration"
CONF_ADV_GAP = "adv_gap"
CONF_MAX_QUEUE_SIZE = "max_queue_size"
CONF_MEMBERSHIP_RETRIES = "membership_retries"
CONF_MEMBERSHIP_TTL = "membership_ttl"
CONF_GROUP_SLOT = "group_slot"
CONF_SNIFFER = "sniffer"
CONF_COMMAND_RETRIES = "command_retries"
CONF_GROUP_SETTLE = "group_settle"
CONF_RETRANSMIT_ENABLED = "retransmit_enabled"
CONF_RETRANSMIT_DELAYS = "retransmit_delays"
CONF_SKIP_TRACKED_MEMBERSHIP = "skip_tracked_membership"
CONF_MEMBERSHIP_REPEAT_GAP = "membership_repeat_gap"

DEFAULT_ADV_INTERVAL_MIN = 0x20
DEFAULT_ADV_INTERVAL_MAX = 0x40
DEFAULT_ADV_DURATION = 50
DEFAULT_ADV_GAP = 10
DEFAULT_MAX_QUEUE_SIZE = 100
DEFAULT_MEMBERSHIP_RETRIES = 3
DEFAULT_MEMBERSHIP_TTL = "30s"

# The id the BRMesh app itself uses for ad-hoc multi-selections. Proven to work on real
# hardware, which an id the app never issues is not.
DEFAULT_GROUP_SLOT = 0xFD

# Off by default: it makes esp32_ble_tracker a hard requirement, and it is only
# useful when something OTHER than this node also commands the mesh.
DEFAULT_SNIFFER = False

# Reception on this protocol is roughly 80%% per advertisement, so a single shot loses a
# bulb about one time in five. Membership writes already repeated; control frames did not.
DEFAULT_COMMAND_RETRIES = 3

# Pause bracketing each membership write, so a bulb is not reassigned to another group
# while it is still acting on the frame before it.
DEFAULT_GROUP_SETTLE = "250ms"

# The +1s/+5s individual-retransmit safety net (schedule_retransmits() in
# fastcon_controller.cpp) - a follow-up resend of the last command for a target, fired
# once the queue goes idle, independent of command_retries_/membership_retries_'s
# same-dispatch back-to-back repeats above. On by default, matching the firmware's
# original fixed behavior.
DEFAULT_RETRANSMIT_ENABLED = True
DEFAULT_RETRANSMIT_DELAYS = ["1s", "5s"]

# ensure_group()'s "skip the membership write if every member is already tracked as
# this group" optimization - on by default, matching the firmware's current behavior.
# Set to false to force every group dispatch to always rewrite membership for every
# member, unconditionally (the pre-2026-09-07 behavior) - see
# FastconController::set_skip_tracked_membership()'s own comment (fastcon_controller.h)
# for why this exists: live testing the night of 2026-09-07 repeatedly showed the skip
# path landing on only 0-2 of a group's 3 members while a forced rewrite landed on all
# 3 every time it was tried.
DEFAULT_SKIP_TRACKED_MEMBERSHIP = True

# Pause between each repeat of a membership write within the same dispatch - 0 (default)
# preserves the old back-to-back-at-the-adv-duty-cycle behavior. Only paces
# GROUP_MEMBERSHIP repeats (membership_retries above), not control-frame repeats
# (command_retries) - see FastconController::set_membership_repeat_gap()'s own comment
# (fastcon_controller.h).
DEFAULT_MEMBERSHIP_REPEAT_GAP = "0ms"


def validate_hex_bytes(value):
    if isinstance(value, str):
        value = value.replace(" ", "")
        if len(value) != 8:
            raise cv.Invalid("Mesh key must be exactly 4 bytes (8 hex characters)")

        try:
            return HexInt(int(value, 16))
        except ValueError as err:
            raise cv.Invalid(f"Invalid hex value: {err}")
    raise cv.Invalid("Mesh key must be a string")


fastcon_ns = cg.esphome_ns.namespace("fastcon")
FastconController = fastcon_ns.class_("FastconController", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ID, default="fastcon_controller"): cv.declare_id(
            FastconController
        ),
        cv.Required(CONF_MESH_KEY): validate_hex_bytes,
        cv.Optional(
            CONF_ADV_INTERVAL_MIN, default=DEFAULT_ADV_INTERVAL_MIN
        ): cv.uint16_t,
        cv.Optional(
            CONF_ADV_INTERVAL_MAX, default=DEFAULT_ADV_INTERVAL_MAX
        ): cv.uint16_t,
        cv.Optional(CONF_ADV_DURATION, default=DEFAULT_ADV_DURATION): cv.uint16_t,
        cv.Optional(CONF_ADV_GAP, default=DEFAULT_ADV_GAP): cv.uint16_t,
        cv.Optional(
            CONF_MAX_QUEUE_SIZE, default=DEFAULT_MAX_QUEUE_SIZE
        ): cv.positive_int,
        cv.Optional(
            CONF_MEMBERSHIP_RETRIES, default=DEFAULT_MEMBERSHIP_RETRIES
        ): cv.int_range(min=1, max=10),
        cv.Optional(
            CONF_MEMBERSHIP_TTL, default=DEFAULT_MEMBERSHIP_TTL
        ): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_GROUP_SLOT, default=DEFAULT_GROUP_SLOT): cv.int_range(
            min=1, max=255
        ),
        # Optional - live test of whether bracketing a group action with a cmd-9
        # time-sync frame (matching the app's own observed habit) improves membership-
        # write/group-control reception. No effect on single-light entities.
        cv.Optional(CONF_TIME_ID): cv.use_id(time_.RealTimeClock),
        # Listen for commands sent by OTHER controllers on this mesh - the phone app,
        # a FastCon scene switch - and publish them onto the matching entities. The
        # bulbs themselves never report anything, so this is the only way Home
        # Assistant learns about a change it did not make. Off by default: it makes
        # esp32_ble_tracker a hard requirement and is pointless on a mesh this node
        # is the sole controller of.
        cv.Optional(CONF_SNIFFER, default=DEFAULT_SNIFFER): cv.boolean,
        cv.Optional(
            CONF_COMMAND_RETRIES, default=DEFAULT_COMMAND_RETRIES
        ): cv.int_range(min=1, max=10),
        cv.Optional(
            CONF_GROUP_SETTLE, default=DEFAULT_GROUP_SETTLE
        ): cv.positive_time_period_milliseconds,
        cv.Optional(
            CONF_RETRANSMIT_ENABLED, default=DEFAULT_RETRANSMIT_ENABLED
        ): cv.boolean,
        cv.Optional(
            CONF_RETRANSMIT_DELAYS, default=DEFAULT_RETRANSMIT_DELAYS
        ): cv.ensure_list(cv.positive_time_period_milliseconds),
        cv.Optional(
            CONF_SKIP_TRACKED_MEMBERSHIP, default=DEFAULT_SKIP_TRACKED_MEMBERSHIP
        ): cv.boolean,
        cv.Optional(
            CONF_MEMBERSHIP_REPEAT_GAP, default=DEFAULT_MEMBERSHIP_REPEAT_GAP
        ): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA).extend(esp32_ble_tracker.ESP_BLE_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_MESH_KEY in config:
        mesh_key = config[CONF_MESH_KEY]
        key_bytes = [(mesh_key >> (i * 8)) & 0xFF for i in range(3, -1, -1)]
        cg.add(var.set_mesh_key(key_bytes))

    if config[CONF_ADV_INTERVAL_MAX] < config[CONF_ADV_INTERVAL_MIN]:
        raise cv.Invalid(
            f"adv_interval_max ({config[CONF_ADV_INTERVAL_MAX]}) must be >= "
            f"adv_interval_min ({config[CONF_ADV_INTERVAL_MIN]})"
        )

    cg.add(var.set_adv_interval_min(config[CONF_ADV_INTERVAL_MIN]))
    cg.add(var.set_adv_interval_max(config[CONF_ADV_INTERVAL_MAX]))
    cg.add(var.set_adv_duration(config[CONF_ADV_DURATION]))
    cg.add(var.set_adv_gap(config[CONF_ADV_GAP]))
    cg.add(var.set_max_queue_size(config[CONF_MAX_QUEUE_SIZE]))
    cg.add(var.set_membership_retries(config[CONF_MEMBERSHIP_RETRIES]))
    cg.add(var.set_membership_ttl(config[CONF_MEMBERSHIP_TTL]))
    cg.add(var.set_group_slot(config[CONF_GROUP_SLOT]))
    cg.add(var.set_command_retries(config[CONF_COMMAND_RETRIES]))
    cg.add(var.set_group_settle(config[CONF_GROUP_SETTLE]))
    cg.add(var.set_retransmit_enabled(config[CONF_RETRANSMIT_ENABLED]))
    cg.add(
        var.set_retransmit_delays(
            [int(delay.total_milliseconds) for delay in config[CONF_RETRANSMIT_DELAYS]]
        )
    )
    cg.add(var.set_skip_tracked_membership(config[CONF_SKIP_TRACKED_MEMBERSHIP]))
    cg.add(var.set_membership_repeat_gap(config[CONF_MEMBERSHIP_REPEAT_GAP]))

    if config[CONF_SNIFFER]:
        cg.add(var.set_sniffer_enabled(True))
        await esp32_ble_tracker.register_ble_device(var, config)

    if CONF_TIME_ID in config:
        time_var = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_source(time_var))


# fastcon.define_group - the only way to provision a group (2026-09-07, per direct request
# "explicit calls to create a group can be reached via the api").
#
# Commanding a group no longer defines it: queueGroupCommand() used to call ensure_group() on
# every group command, which put the one fragile step of the whole dispatch on the critical
# path of every command. The phone app provisions a group once, when you create it, and
# afterwards sends nothing but cmd 3 - see DefineGroupAction (fastcon_controller.h).
#
#   api:
#     actions:
#       - action: define_group
#         variables:
#           group_id: int
#           members: int[]
#         then:
#           - fastcon.define_group:
#               group_id: !lambda 'return group_id;'
#               members: !lambda 'return members;'
#
# An empty `members` list dissolves the group. group_id 0 is rejected here rather than at
# runtime - it is the firmware's own all-lights group and has no membership to write.
CONF_GROUP_ID = "group_id"
CONF_MEMBERS = "members"
CONF_CONTROLLER_ID = "controller_id"
MAX_LIGHT_ID = 255

DefineGroupAction = fastcon_ns.class_("DefineGroupAction", automation.Action)


@automation.register_action(
    "fastcon.define_group",
    DefineGroupAction,
    cv.Schema(
        {
            cv.Optional(CONF_CONTROLLER_ID, default="fastcon_controller"): cv.use_id(
                FastconController
            ),
            cv.Required(CONF_GROUP_ID): cv.templatable(
                cv.int_range(min=1, max=MAX_LIGHT_ID)
            ),
            cv.Required(CONF_MEMBERS): cv.templatable(
                cv.ensure_list(cv.int_range(min=1, max=MAX_LIGHT_ID))
            ),
        }
    ),
    # play() only packs a mask and queues frames - nothing is deferred to a timer, callback
    # or loop(), so play_next_() always runs before play_complex() returns.
    synchronous=True,
)
async def define_group_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_CONTROLLER_ID])
    var = cg.new_Pvariable(action_id, template_arg, parent)
    group_id = await cg.templatable(config[CONF_GROUP_ID], args, cg.uint8)
    cg.add(var.set_group_id(group_id))
    members = await cg.templatable(
        config[CONF_MEMBERS], args, cg.std_vector.template(cg.int32)
    )
    cg.add(var.set_members(members))
    return var
