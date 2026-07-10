import json

import esphome.codegen as cg
from esphome import automation
from esphome import pins
from esphome.components import sensor, text_sensor
import esphome.config_validation as cv
from esphome.core import CORE
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_PIN,
    CONF_UPDATE_INTERVAL,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_PERCENT,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

CODEOWNERS = ["@custom"]
AUTO_LOAD = ["json", "web_server_base", "sensor", "text_sensor"]
DEPENDENCIES = ["esp32", "network"]

CONF_API_PATH = "api_path"
CONF_CHANNELS = "channels"
CONF_CURVE = "curve"
CONF_DEFAULT_PWM = "default_pwm"
CONF_DATA_TIMEOUT = "data_timeout"
CONF_HYSTERESIS = "hysteresis"
CONF_INVERTED = "inverted"
CONF_LEDC_CHANNEL = "ledc_channel"
CONF_MANUAL_PWM = "manual_pwm"
CONF_MAX_PWM = "max_pwm"
CONF_MIN_PWM = "min_pwm"
CONF_PWM = "pwm"
CONF_PWM_FREQUENCY = "pwm_frequency"
CONF_SOURCE = "source"
CONF_TEMP = "temp"
CONF_UI_PATH = "ui_path"
CONF_VALUE = "value"
CONF_TEMPERATURE_SENSOR = "temperature_sensor"
CONF_PWM_SENSOR = "pwm_sensor"
CONF_POLLING_ENABLED = "polling_enabled"
CONF_STATUS_TEXT_SENSOR = "status_text_sensor"

pc_fan_controller_ns = cg.esphome_ns.namespace("pc_fan_controller")
PcFanController = pc_fan_controller_ns.class_("PcFanController", cg.Component)
SetTemperatureAction = pc_fan_controller_ns.class_(
    "SetTemperatureAction", automation.Action.template()
)

_PC_FAN_CONTROLLER_INSTANCES = []

PWM_PERCENT = cv.All(cv.float_, cv.Range(min=0.0, max=100.0))
TEMP_VALUE = cv.All(cv.float_, cv.Range(min=-40.0, max=150.0))

CURVE_POINT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_TEMP): TEMP_VALUE,
        cv.Required(CONF_PWM): PWM_PERCENT,
    }
)

CHANNEL_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_NAME): cv.All(cv.string, cv.Length(min=1, max=31)),
        cv.Required(CONF_PIN): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_LEDC_CHANNEL): cv.int_range(min=0, max=7),
        cv.Optional(CONF_INVERTED, default=True): cv.boolean,
        cv.Optional(CONF_SOURCE, default="max"): cv.one_of(
            "cpu",
            "gpu",
            "other",
            "max",
            lower=True,
        ),
        cv.Optional(CONF_MIN_PWM, default=25.0): PWM_PERCENT,
        cv.Optional(CONF_MAX_PWM, default=100.0): PWM_PERCENT,
        cv.Optional(CONF_DEFAULT_PWM, default=50.0): PWM_PERCENT,
        cv.Optional(CONF_MANUAL_PWM, default=50.0): PWM_PERCENT,
        cv.Optional(CONF_HYSTERESIS, default=3.0): cv.All(
            cv.float_,
            cv.Range(min=0.0, max=20.0),
        ),
        cv.Optional(
            CONF_CURVE,
            default=[
                {CONF_TEMP: 35.0, CONF_PWM: 25.0},
                {CONF_TEMP: 50.0, CONF_PWM: 35.0},
                {CONF_TEMP: 65.0, CONF_PWM: 60.0},
                {CONF_TEMP: 80.0, CONF_PWM: 100.0},
            ],
        ): cv.All(
            cv.ensure_list(CURVE_POINT_SCHEMA),
            cv.Length(min=2, max=8),
        ),
        cv.Optional(CONF_TEMPERATURE_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_PWM_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_STATUS_TEXT_SENSOR): text_sensor.text_sensor_schema(
            icon="mdi:fan",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)


def validate_channels(config):
    channels = config[CONF_CHANNELS]

    used_ledc_channels = set()
    for channel in channels:
        ledc_channel = channel[CONF_LEDC_CHANNEL]
        if ledc_channel in used_ledc_channels:
            raise cv.Invalid(f"Duplicate LEDC channel: {ledc_channel}")
        used_ledc_channels.add(ledc_channel)

        if channel[CONF_MIN_PWM] > channel[CONF_MAX_PWM]:
            raise cv.Invalid(
                f"{channel[CONF_NAME]}: min_pwm must not be higher than max_pwm"
            )

    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PcFanController),
            cv.Optional(CONF_UPDATE_INTERVAL, default="500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DATA_TIMEOUT, default="10s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PWM_FREQUENCY, default=25000): cv.int_range(
                min=10,
                max=300000,
            ),
            cv.Optional(CONF_UI_PATH, default="/fan-control"): cv.All(
                cv.string,
                cv.Length(min=1, max=40),
            ),
            cv.Optional(CONF_API_PATH, default="/fan-api"): cv.All(
                cv.string,
                cv.Length(min=1, max=40),
            ),
            cv.Optional(CONF_POLLING_ENABLED, default=True): cv.boolean,
            cv.Required(CONF_CHANNELS): cv.All(
                cv.ensure_list(CHANNEL_SCHEMA),
                cv.Length(min=1, max=8),
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_channels,
    cv.only_on_esp32,
)


async def to_code(config):
    channels = config[CONF_CHANNELS]

    cg.add_define("USE_PC_FAN_CONTROLLER_MAX_CHANNELS", len(channels))

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    _PC_FAN_CONTROLLER_INSTANCES.append(var)

    cg.add(var.set_update_interval_ms(config[CONF_UPDATE_INTERVAL].total_milliseconds))
    cg.add(var.set_data_timeout_ms(config[CONF_DATA_TIMEOUT].total_milliseconds))
    cg.add(var.set_pwm_frequency(config[CONF_PWM_FREQUENCY]))
    cg.add(var.set_ui_path(config[CONF_UI_PATH]))
    cg.add(var.set_api_path(config[CONF_API_PATH]))
    cg.add(var.set_polling_enabled(config[CONF_POLLING_ENABLED]))

    ui_title = CORE.friendly_name or CORE.name
    cg.add(var.set_ui_title(ui_title))

    for index, channel in enumerate(channels, start=1):
        pin = await cg.gpio_pin_expression(channel[CONF_PIN])
        curve_json = json.dumps(channel[CONF_CURVE], separators=(",", ":"))

        cg.add(
            var.add_channel(
                pin,
                channel[CONF_NAME],
                channel[CONF_LEDC_CHANNEL],
                channel[CONF_INVERTED],
                channel[CONF_SOURCE],
                channel[CONF_MIN_PWM],
                channel[CONF_MAX_PWM],
                channel[CONF_DEFAULT_PWM],
                channel[CONF_MANUAL_PWM],
                channel[CONF_HYSTERESIS],
                curve_json,
            )
        )

        if CONF_TEMPERATURE_SENSOR in channel:
            sens = await sensor.new_sensor(channel[CONF_TEMPERATURE_SENSOR])
            cg.add(var.set_channel_temperature_sensor(index, sens))

        if CONF_PWM_SENSOR in channel:
            sens = await sensor.new_sensor(channel[CONF_PWM_SENSOR])
            cg.add(var.set_channel_pwm_sensor(index, sens))

        if CONF_STATUS_TEXT_SENSOR in channel:
            sens = await text_sensor.new_text_sensor(channel[CONF_STATUS_TEXT_SENSOR])
            cg.add(var.set_channel_status_text_sensor(index, sens))


@automation.register_action(
    "pc_fan_controller.set_temperature",
    SetTemperatureAction,
    schema=cv.Schema(
        {
            cv.Optional(CONF_ID): cv.use_id(PcFanController),
            cv.Required(CONF_SOURCE): cv.one_of("cpu", "gpu", "other", lower=True),
            cv.Required(CONF_VALUE): cv.templatable(cv.float_),
        }
    ),
    synchronous=True,
)
async def pc_fan_controller_set_temperature_action_to_code(
    config, action_id, template_arg, args
):
    if CONF_ID in config:
        paren = await cg.get_variable(config[CONF_ID])
    else:
        if len(_PC_FAN_CONTROLLER_INSTANCES) != 1:
            raise cv.Invalid(
                "pc_fan_controller.set_temperature requires id when multiple controllers are defined"
            )
        paren = _PC_FAN_CONTROLLER_INSTANCES[0]

    var = cg.new_Pvariable(action_id, template_arg, paren)
    cg.add(var.set_source(config[CONF_SOURCE]))
    template_ = await cg.templatable(config[CONF_VALUE], args, float)
    cg.add(var.set_value(template_))
    return var
