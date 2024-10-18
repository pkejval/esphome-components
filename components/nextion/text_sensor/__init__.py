from esphome.components import text_sensor
import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID

from .. import nextion_ns, CONF_NEXTION_ID

from ..base_component import (
    setup_component_core_,
    CONFIG_TEXT_COMPONENT_SCHEMA,
)

CODEOWNERS = ["@senexcrenshaw"]

NextionTextSensor = nextion_ns.class_(
    "NextionTextSensor", text_sensor.TextSensor, cg.PollingComponent
)

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(NextionTextSensor)
    .extend(CONFIG_TEXT_COMPONENT_SCHEMA)
    .extend(cv.polling_component_schema("never"))
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_NEXTION_ID])
    var = cg.new_Pvariable(config[CONF_ID], hub)
    await text_sensor.register_text_sensor(var, config)
    await cg.register_component(var, config)

    cg.add(hub.register_textsensor_component(var))

    await setup_component_core_(var, config, ".txt")

@automation.register_action(
    "text_sensor.nextion.publish",
    text_sensor.TextSensorPublishAction,
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.use_id(text_sensor.NextionTextSensor),
            cv.Required(CONF_STATE): cv.templatable(cv.float_),
            cv.Optional(CONF_PUBLISH_STATE, default="true"): cv.templatable(cv.boolean),
            cv.Optional(CONF_SEND_TO_NEXTION, default="true"): cv.templatable(cv.boolean)
        }
    ),
)
async def text_sensor_nextion_publish_to_code(config, action_id, template_arg, args):
    paren = await cg.get_variable(config[CONF_ID])
    var = cg.new_Pvariable(action_id, template_arg, paren)
    template_state = await cg.templatable(config[CONF_STATE], args, float)
    template_publish = await cg.templatable(config[CONF_PUBLISH_STATE], args, bool)
    template_send_to_nextion = await cg.templatable(config[CONF_SEND_TO_NEXTION], args, bool)
    cg.add(var.set_state(template_state, template_publish, template_send_to_nextion))
    return var