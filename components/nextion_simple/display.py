import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import uart
from esphome.const import CONF_ID, CONF_UART_ID, CONF_TRIGGER_ID

DEPENDENCIES = ["uart"]

ns = cg.esphome_ns.namespace("nextion_simple")
NextionSimple = ns.class_("NextionSimple", cg.Component)

# Triggery
NextionSetupTrigger = ns.class_("NextionSetupTrigger", automation.Trigger.template())
NextionPageTrigger = ns.class_("NextionPageTrigger", automation.Trigger.template())
NextionReadyTrigger = ns.class_("NextionReadyTrigger", automation.Trigger.template())

# C++ Action třídy
SetComponentValueAction = ns.class_(
    "SetComponentValueAction", automation.Action.template()
)
SetComponentTextAction = ns.class_(
    "SetComponentTextAction", automation.Action.template()
)
SetComponentPiccAction = ns.class_(
    "SetComponentPiccAction", automation.Action.template()
)
SetComponentPicc1Action = ns.class_(
    "SetComponentPicc1Action", automation.Action.template()
)
SetComponentBackgroundColorAction = ns.class_(
    "SetComponentBackgroundColorAction", automation.Action.template()
)
SetComponentFontColorAction = ns.class_(
    "SetComponentFontColorAction", automation.Action.template()
)
SetComponentVisibilityAction = ns.class_(
    "SetComponentVisibilityAction", automation.Action.template()
)
SetPageAction = ns.class_("SetPageAction", automation.Action.template())

# Auto-target registr instancí
_NEXTION_INSTANCES = []

# Keys
CONF_TFT_URL = "tft_url"
CONF_ON_SETUP = "on_setup"
CONF_ON_PAGE = "on_page"
CONF_ON_NEXTION_READY = "on_nextion_ready"
CONF_NEXTION_READY_COOLDOWN = "nextion_ready_cooldown"

CONF_PAGE = "page"
CONF_COMPONENT = "component"
CONF_VALUE = "value"
CONF_TEXT = "text"
CONF_COLOR = "color"
CONF_STATE = "state"

# ============= CONFIG =============
CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NextionSimple),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_TFT_URL): cv.string,
        cv.Optional(CONF_ON_SETUP): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionSetupTrigger),
            }
        ),
        cv.Optional(CONF_ON_PAGE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionPageTrigger),
            }
        ),
        cv.Optional(CONF_ON_NEXTION_READY): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(NextionReadyTrigger),
            }
        ),
        cv.Optional(
            CONF_NEXTION_READY_COOLDOWN, default="500ms"
        ): cv.positive_time_period_milliseconds,
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    _NEXTION_INSTANCES.append(var)
    await cg.register_component(var, config)

    uart_par = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_par))

    if CONF_TFT_URL in config:
        cg.add(var.set_tft_url(config[CONF_TFT_URL]))
    if "nextion_ready_cooldown" in config:
        cg.add(
            var.set_nextion_ready_cooldown(
                int(config["nextion_ready_cooldown"].total_milliseconds)
            )
        )

    if "on_setup" in config:
        for conf in config["on_setup"]:
            trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trig, [], conf)
    if "on_page" in config:
        for conf in config["on_page"]:
            trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trig, [(cg.int_, "page")], conf)
    if "on_nextion_ready" in config:
        for conf in config["on_nextion_ready"]:
            trig = cg.new_Pvariable(conf[CONF_TRIGGER_ID], var)
            await automation.build_automation(trig, [], conf)
