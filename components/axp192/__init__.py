import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.components import i2c, sensor, binary_sensor
from esphome.const import CONF_ID,\
    CONF_BATTERY_LEVEL, CONF_BRIGHTNESS, UNIT_PERCENT, ICON_BATTERY, CONF_MODEL

#CONF_CHARGING = "charging"
CONF_AXP192_ID = 'axp192_id'
DEPENDENCIES = ['i2c']
axp192_ns = cg.esphome_ns.namespace('axp192')
AXP192Component = axp192_ns.class_('AXP192Component', cg.PollingComponent, i2c.I2CDevice)
#AXP192Model = axp192_ns.enum("AXP192Model")

AXP192Model = axp192_ns.enum('MODELS', is_class=True)
MODELS = {
    'M5CORE2': AXP192Model.M5CORE2,
    'M5STICKC': AXP192Model.M5STICKC,
    'M5TOUGH': AXP192Model.M5TOUGH,
}

CONF_OUTPUT_LDO2 = 'ldo2'
CONF_OUTPUT_LDO3 = 'ldo3'
CONF_OUTPUT_DCDC1 = 'dcdc1'
CONF_OUTPUT_DCDC3 = 'dcdc3'
CONF_OUTPUT_LDOIO0 = 'ldoio0'

output_pin = axp192_ns.enum('OutputPin', is_class=True)
OUTPUT_PIN = {
    CONF_OUTPUT_LDO2: output_pin.OUTPUT_LDO2,
    CONF_OUTPUT_LDO3: output_pin.OUTPUT_LDO3,
    CONF_OUTPUT_DCDC1: output_pin.OUTPUT_DCDC1,
    CONF_OUTPUT_DCDC3: output_pin.OUTPUT_DCDC3,
    CONF_OUTPUT_LDOIO0: output_pin.OUTPUT_LDOIO0,
}

VoffVoltage = axp192_ns.enum('VoffVoltage', is_class=True)
VOFFVOLTAGE = {
    '2600mV': VoffVoltage.VOFF_2600MV,
    '2700mV': VoffVoltage.VOFF_2700MV,
    '2800mV': VoffVoltage.VOFF_2800MV,
    '2900mV': VoffVoltage.VOFF_2900MV,
    '3000mV': VoffVoltage.VOFF_3000MV,
    '3100mV': VoffVoltage.VOFF_3100MV,
    '3200mV': VoffVoltage.VOFF_3200MV,
    '3300mV': VoffVoltage.VOFF_3300MV,
}
CONF_VOFF_VOLTAGE = 'voff_voltage'

ChargeCurrent = axp192_ns.enum('ChargeCurrent', is_class=True)
CHARGECURRENT = {
    '100mA': ChargeCurrent.CHARGE_100MA,
    '190mA': ChargeCurrent.CHARGE_190MA,
    '280mA': ChargeCurrent.CHARGE_280MA,
    '360mA': ChargeCurrent.CHARGE_360MA,
    '450mA': ChargeCurrent.CHARGE_450MA,
    '550mA': ChargeCurrent.CHARGE_550MA,
    '630mA': ChargeCurrent.CHARGE_630MA,
    '700mA': ChargeCurrent.CHARGE_700MA,
}
CONF_CHARGE_CURRENT = 'charge_current'

ChargeVoltage = axp192_ns.enum('ChargeVoltage', is_class=True)
CHARGEVOLTAGE = {
    '4100mV': ChargeVoltage.CHARGE_4100MV,
    '4150mV': ChargeVoltage.CHARGE_4150MV,
    '4200mV': ChargeVoltage.CHARGE_4200MV,
    '4360mV': ChargeVoltage.CHARGE_4360MV,
}
CONF_CHARGE_VOLTAGE = 'charge_voltage'

VBusHoldVoltageLimit = axp192_ns.enum('VBusHoldVoltageLimit', is_class=True)
VBUS_HOLD_VOLTAGE_LIMIT = {
    '4000mV': VBusHoldVoltageLimit.HOLD_4000MV,
    '4100mV': VBusHoldVoltageLimit.HOLD_4100MV,
    '4200mV': VBusHoldVoltageLimit.HOLD_4200MV,
    '4300mV': VBusHoldVoltageLimit.HOLD_4300MV,
    '4400mV': VBusHoldVoltageLimit.HOLD_4400MV,
    '4500mV': VBusHoldVoltageLimit.HOLD_4500MV,
    '4600mV': VBusHoldVoltageLimit.HOLD_4600MV,
    '4700mV': VBusHoldVoltageLimit.HOLD_4700MV,
}
CONF_VBUS_HOLD_VOLTAGE_LIMIT = 'vbus_hold_voltage_limit'

VBusHoldVoltageLimited = axp192_ns.enum('VBusHoldVoltageLimited', is_class=True)
VBUS_HOLD_VOLTAGE_LIMITED = {
    'NO': VBusHoldVoltageLimited.VOLTAGE_NOT_LIMITED,
    'YES': VBusHoldVoltageLimited.VOLTAGE_LIMITED,
}
CONF_VBUS_HOLD_VOLTAGE_LIMITED = 'vbus_hold_voltage_limited'

VBusHoldCurrentLimit = axp192_ns.enum('VBusHoldCurrentLimit', is_class=True)
VBUS_HOLD_CURRENT_LIMIT = {
    '500mA': VBusHoldCurrentLimit.CURRENT_LIMIT_500MA,
    '100mA': VBusHoldCurrentLimit.CURRENT_LIMIT_100MA,
}
CONF_VBUS_HOLD_CURRENT_LIMIT = 'vbus_hold_current_limit'

VBusHoldCurrentLimited = axp192_ns.enum('VBusHoldCurrentLimited', is_class=True)
VBUS_HOLD_CURRENT_LIMITED = {
    'NO': VBusHoldCurrentLimited.CURRENT_NOT_LIMITED,
    'YES': VBusHoldCurrentLimited.CURRENT_LIMITED,
}
CONF_VBUS_HOLD_CURRENT_LIMITED = 'vbus_hold_current_limited'

VBusIpsout = axp192_ns.enum('VBusIpsout', is_class=True)
VBUS_IPSOUT = {
    'NO': VBusIpsout.IPSOUT_NOT_MANAGED,
    'YES': VBusIpsout.IPSOUT_MANAGED,
}
CONF_VBUS_IPSOUT = 'vbus_ipsout'

LDOio0Control = axp192_ns.enum('LDOio0Control', is_class=True)
LDOIO0_MODE = {
    'NMOS_OPENDRAIN': LDOio0Control.NMOS_OPENDRAIN,
    'GENERAL_INPUT': LDOio0Control.GENERAL_INPUT,
    'LOWNOISE_LDO': LDOio0Control.LOWNOISE_LDO,
    'ADC_IN': LDOio0Control.ADC_IN,
    'LOW_OUTPUT': LDOio0Control.LOW_OUTPUT,
    'FLOATING': LDOio0Control.FLOATING,

}
CONF_LDOIO0_MODE = 'ldoio0_mode'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(AXP192Component),
    cv.Required(CONF_MODEL): cv.enum(MODELS),
    cv.Optional(CONF_BATTERY_LEVEL):
        sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            icon=ICON_BATTERY,
        ),
    cv.Optional(CONF_CHARGING): binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_BRIGHTNESS, default=1.0): cv.percentage,
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x77))


def to_code(config):
    cg.add_global(axp192_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)

    cg.add(var.set_model(config[CONF_MODEL]))
    if CONF_BATTERY_LEVEL in config:
        conf = config[CONF_BATTERY_LEVEL]
        sens = yield sensor.new_sensor(conf)
        cg.add(var.set_batterylevel_sensor(sens))
    if CONF_CHARGING in config:
        conf = config[CONF_CHARGING]
        sens = yield binary_sensor.new_binary_sensor(conf)
        cg.add(var.set_charging_sensor(sens))
    if CONF_BRIGHTNESS in config:
        conf = config[CONF_BRIGHTNESS]
        cg.add(var.set_brightness(conf))
