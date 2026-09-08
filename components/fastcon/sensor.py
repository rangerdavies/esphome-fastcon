"""Sensor platform exposing each bulb's own reported mesh group.

The value comes from that bulb's heartbeat broadcast, not from anything this component sent,
so it answers "where is this bulb actually" rather than "where did we ask it to be". See
FastconGroupSensor's own comment (fastcon_group_sensor.h).

    sensor:
      - platform: fastcon
        light_id: 1
        name: "Living Room Light 5 Group"
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from .fastcon_controller import FastconController, fastcon_ns

DEPENDENCIES = ["esp32_ble"]

CONF_LIGHT_ID = "light_id"
CONF_CONTROLLER_ID = "controller_id"

MAX_LIGHT_ID = 255

FastconGroupSensor = fastcon_ns.class_("FastconGroupSensor", sensor.Sensor, cg.Component)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        FastconGroupSensor,
        accuracy_decimals=0,
        icon="mdi:lightbulb-group",
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )
    .extend(
        {
            cv.Required(CONF_LIGHT_ID): cv.int_range(min=1, max=MAX_LIGHT_ID),
            cv.Optional(CONF_CONTROLLER_ID, default="fastcon_controller"): cv.use_id(
                FastconController
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    cg.add(var.set_light_id(config[CONF_LIGHT_ID]))
    # Registered from here rather than from the sensor's own setup(), which would need the
    # full controller type in fastcon_group_sensor.h and make the two headers mutually
    # dependent. The controller only ever forward-declares this class.
    controller = await cg.get_variable(config[CONF_CONTROLLER_ID])
    cg.add(controller.register_group_sensor(var))
