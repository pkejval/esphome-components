import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.core import CORE
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    CONF_NUMBER,
    CONF_INTERNAL_FILTER,
    CONF_VALUE,
    ICON_PULSE,
    STATE_CLASS_MEASUREMENT,
    UNIT_PULSES_PER_MINUTE,
    UNIT_PULSES,
)

CONF_USE_HFLOOP = "use_hfloop"

pulse_counter_ns = cg.esphome_ns.namespace("custom_pulse_counter")

CustomPulseCounter = pulse_counter_ns.class_(
    "CustomPulseCounter", sensor.Sensor, cg.PollingComponent
)

def validate_internal_filter(value):
    return value

def validate_pulse_counter_pin(value):
    value = pins.internal_gpio_input_pin_schema(value)
    return value

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        CustomPulseCounter,
        unit_of_measurement=UNIT_PULSES_PER_MINUTE,
        icon=ICON_PULSE,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Required(CONF_PIN): validate_pulse_counter_pin,
            cv.Optional(
                CONF_INTERNAL_FILTER, default="13us"
            ): cv.positive_time_period_microseconds,
            cv.Optional(CONF_USE_HFLOOP, default="true"): cv.boolean,
        },
    )
    .extend(cv.polling_component_schema("500ms")),
    validate_internal_filter,
)

async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_filter_us(config[CONF_INTERNAL_FILTER]))