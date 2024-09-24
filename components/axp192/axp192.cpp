#include "axp192.h"

#ifdef USE_BINARY_SENSOR
#include "binary_sensor/axp192_binary_sensor.h"
#endif

#ifdef USE_OUTPUT
#include "output/axp192_output.h"
#endif

#ifdef USE_SENSOR
#include "sensor/axp192_sensor.h"
#endif

#ifdef USE_SWITCH
#include "switch/axp192_switch.h"
#endif

#include "esphome/core/log.h"
//#include "esp_sleep.h"
#include <Esp.h>
#define TAG this->get_component_source()

namespace esphome {
namespace axp192 {

  namespace detail {

    // NOLINTNEXTLINE(readability-identifier-naming)
    const char *const bit_rep[16] = {
        [0] = "0000",  [1] = "0001",  [2] = "0010",  [3] = "0011",  [4] = "0100",  [5] = "0101",
        [6] = "0110",  [7] = "0111",  [8] = "1000",  [9] = "1001",  [10] = "1010", [11] = "1011",
        [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
    };

    std::string format_bits(uint8_t byte) { return std::string{"0b"} + bit_rep[byte >> 4] + bit_rep[byte & 0x0F]; }

    template<size_t N>
    void log_register_bits(const char *const tag, RegisterLocations reg, const std::array<uint8_t, N> &bytes) {
      auto regnum = detail::to_int(reg);
      ESP_LOGV(tag, "Register bytes starting at 0x%02X", regnum);
      for (int idx = 0; idx < N; idx++) {
        ESP_LOGV(tag, "  Register 0x%02X is %s", idx + regnum, detail::format_bits(bytes[idx]).c_str());
      }
    }

    constexpr uint16_t encode_12bit(uint8_t msb, uint8_t lsb) {
      return (static_cast<uint16_t>(msb) << 4) | (static_cast<uint16_t>(lsb));
    }

    constexpr uint16_t encode_13bit(uint8_t msb, uint8_t lsb) {
      return (static_cast<uint16_t>(msb) << 4) | (static_cast<uint16_t>(lsb));
    }

  }

void AXP192Component::setup()
{
  switch (this->model_) {
    case AXP192Model::M5STICKC:
    {
        begin(false, false, false, false, false);
        break;
    }
    case AXP192Model::M5CORE2:
    {
        // disable LDO3 Vibration
        begin(false, true, false, false, false);
        break;
    }
    case AXP192Model::M5TOUGH:
    {
        begin(false, false, false, false, false);

        // If we're waking from a cold boot
        /*if (GetStartupReason() == "ESP_RST_POWERON")
        {
            ESP_LOGD(TAG, "First power on, restarting ESP...");

            // Reboot the ESP with the axp initialised
            ESP.restart();
        }*/
        break;
    }
  }
  if (!this->configure_axp()) {
    this->mark_failed();
  }
}

void AXP192Component::update() {
    this->debug_log_register_(RegisterLocations::DCDC13_LDO23_CONTROL);
    this->debug_log_register_(RegisterLocations::DCDC2_VOLTAGE);
    this->debug_log_register_(RegisterLocations::DCDC1_VOLTAGE);
    this->debug_log_register_(RegisterLocations::DCDC3_VOLTAGE);
    this->debug_log_register_(RegisterLocations::LDO23_VOLTAGE);
    this->debug_log_register_(RegisterLocations::GPIO_LDO_VOLTAGE);
    this->update_powercontrol(OutputPin::OUTPUT_LDO2, this->get_ldo2_enabled());
    this->update_powercontrol(OutputPin::OUTPUT_LDO3, this->get_ldo3_enabled());
    this->update_powercontrol(OutputPin::OUTPUT_DCDC1, this->get_dcdc1_enabled());
    this->update_powercontrol(OutputPin::OUTPUT_DCDC3, this->get_dcdc3_enabled());
    this->update_powercontrol(OutputPin::OUTPUT_LDOIO0, this->get_ldoio0_enabled());

#ifdef USE_BINARY_SENSOR
  if (!this->monitors_.empty()) {
    {
      auto buffer = this->read_bytes<4>(0x0);
      if (buffer.has_value()) {
        ESP_LOGV(this->get_component_source(), "Binary sensors");
        detail::log_register_bits(this->get_component_source(), RegisterLocations::POWER_SUPPLY_STATUS, buffer.value());

        // power supply
        bool acin_present = (buffer.value().at(0) & 0b10000000) != 0;

        this->publish_helper_(MonitorType::ACIN_PRESENT, acin_present);

        bool acin_valid = (buffer.value().at(0) & 0b01000000) != 0;
        this->publish_helper_(MonitorType::ACIN_VALID, !acin_valid);

        bool vbus_present = (buffer.value().at(0) & 0b00100000) != 0;
        this->publish_helper_(MonitorType::VBUS_PRESENT, vbus_present);

        bool vbus_valid = (buffer.value().at(0) & 0b00010000) != 0;
        this->publish_helper_(MonitorType::VBUS_VALID, !vbus_valid);

        bool vhold_over = (buffer.value().at(0) & 0b00001000) != 0;
        this->publish_helper_(MonitorType::VBUS_ABOVE, vhold_over);

        bool charge_state = (buffer.value().at(0) & 0b00000100) != 0;
        this->publish_helper_(MonitorType::BATTERY_CURRENT_DIRECTION, !charge_state);

        bool acin_vbus_short = (buffer.value().at(0) & 0b00000010) != 0;
        this->publish_helper_(MonitorType::ACIN_VBUS_SHORT, acin_vbus_short);

        bool acin_vbus_trigger = (buffer.value().at(0) & 0b00000001) != 0;
        this->publish_helper_(MonitorType::ACIN_VBUS_TRIGGER_BOOT, acin_vbus_trigger);

        // battery
        bool axp_overtemp = (buffer.value().at(1) & 0b10000000) != 0;
        this->publish_helper_(MonitorType::AXP_OVER_TEMP, axp_overtemp);

        bool charging = (buffer.value().at(1) & 0b01000000) != 0;
        this->publish_helper_(MonitorType::CHARGE_INDICATE, charging);

        bool battery_present = (buffer.value().at(1) & 0b00100000) != 0;
        this->publish_helper_(MonitorType::BATTERY_PRESENT, battery_present);

        bool battery_active = (buffer.value().at(1) & 0b00001000) != 0;
        this->publish_helper_(MonitorType::BATTERY_ACTIVE, battery_active);

        bool current_under = (buffer.value().at(1) & 0b00000100) != 0;
        this->publish_helper_(MonitorType::CHARGE_CURRENT_LOW, current_under);
      }
    }
  }
#endif
#ifdef USE_SENSOR
  if (!this->sensors_.empty()) {
    ESP_LOGV(this->get_component_source(), "Value sensors:");
    // Scale values from section 9.7
    {
      auto buffer = this->read_bytes<10>(detail::to_int(RegisterLocations::ACIN_VOLTAGE_HIGH8));
      if (buffer.has_value()) {
        detail::log_register_bits(this->get_component_source(), RegisterLocations::ACIN_VOLTAGE_HIGH8, buffer.value());

        auto acin_voltage = detail::encode_12bit(buffer.value().at(0), buffer.value().at(1));
        this->publish_helper_(SensorType::ACIN_VOLTAGE, remap<float, uint16_t>(acin_voltage, 0x0, 0xFFF, 0, 6.9615));

        auto acin_current = detail::encode_12bit(buffer.value().at(2), buffer.value().at(3));
        this->publish_helper_(SensorType::ACIN_CURRENT, remap<float, uint16_t>(acin_current, 0x0, 0xFFF, 0, 2.5594));

        auto vbus_voltage = detail::encode_12bit(buffer.value().at(4), buffer.value().at(5));
        this->publish_helper_(SensorType::VBUS_VOLTAGE, remap<float, uint16_t>(vbus_voltage, 0x0, 0xFFF, 0, 6.9615));

        auto vbus_current = detail::encode_12bit(buffer.value().at(6), buffer.value().at(7));
        this->publish_helper_(SensorType::VBUS_CURRENT, remap<float, uint16_t>(vbus_current, 0x0, 0xFFF, 0, 1.5356));

        auto axp_internal_temp = detail::encode_12bit(buffer.value().at(8), buffer.value().at(9));
        this->publish_helper_(SensorType::AXP_TEMP,
                              remap<float, uint16_t>(axp_internal_temp, 0x0, 0xFFF, -144.7, 264.8));
      }
    }

    {
      auto buffer = this->read_bytes<10>(detail::to_int(RegisterLocations::BATTERY_TEMP_HIGH8));
      if (buffer.has_value()) {
        detail::log_register_bits(this->get_component_source(), RegisterLocations::BATTERY_TEMP_HIGH8, buffer.value());
        auto battery_temp = detail::encode_12bit(buffer.value().at(0), buffer.value().at(1));
        this->publish_helper_(SensorType::BATTERY_TEMP,
                              remap<float, uint16_t>(battery_temp, 0x0, 0xFFF, -144.7, 264.8));

        auto gpio0_voltage = detail::encode_12bit(buffer.value().at(2), buffer.value().at(3));
        this->publish_helper_(SensorType::GPIO0_VOLTAGE, remap<float, uint16_t>(gpio0_voltage, 0x0, 0xFFF, 0, 2.0475));

        auto gpio1_voltage = detail::encode_12bit(buffer.value().at(4), buffer.value().at(5));
        this->publish_helper_(SensorType::GPIO1_VOLTAGE, remap<float, uint16_t>(gpio1_voltage, 0x0, 0xFFF, 0, 2.0475));

        auto gpio2_voltage = detail::encode_12bit(buffer.value().at(6), buffer.value().at(7));
        this->publish_helper_(SensorType::GPIO2_VOLTAGE, remap<float, uint16_t>(gpio2_voltage, 0x0, 0xFFF, 0, 2.0475));

        auto gpio3_voltage = detail::encode_12bit(buffer.value().at(8), buffer.value().at(9));
        this->publish_helper_(SensorType::GPIO3_VOLTAGE, remap<float, uint16_t>(gpio3_voltage, 0x0, 0xFFF, 0, 2.0475));
      }
    }

    {
      auto buffer = this->read_bytes<16>(detail::to_int(RegisterLocations::BATTERY_POWER_HIGH8));
      if (buffer.has_value()) {
        detail::log_register_bits(this->get_component_source(), RegisterLocations::BATTERY_POWER_HIGH8, buffer.value());

        auto battery_power = encode_uint24(buffer.value().at(0), buffer.value().at(1), buffer.value().at(2));
        this->publish_helper_(SensorType::BATTERY_POWER, battery_power);

        auto battery_voltage = detail::encode_12bit(buffer.value().at(8), buffer.value().at(9));
        this->publish_helper_(SensorType::BATTERY_VOLTAGE,
                              remap<float, uint16_t>(battery_voltage, 0x0, 0xFFF, 0, 4.5045));

        auto battery_charge_current = detail::encode_13bit(buffer.value().at(10), buffer.value().at(11));
        this->publish_helper_(SensorType::BATTERY_CHARGE_CURRENT,
                              remap<float, uint16_t>(battery_charge_current, 0x0, 0xFFF, 0, 4.095));

        auto battery_discharge_current = detail::encode_13bit(buffer.value().at(12), buffer.value().at(13));
        this->publish_helper_(SensorType::BATTERY_DISCHARGE_CURRENT,
                              remap<float, uint16_t>(battery_discharge_current, 0x0, 0xFFF, 0, 4.095));

        auto aps_voltage = detail::encode_12bit(buffer.value().at(14), buffer.value().at(15));
        this->publish_helper_(SensorType::APS_VOLTAGE, remap<float, uint16_t>(aps_voltage, 0x0, 0xFFF, 0, 5.733));
      }
    }
  }
#endif

    /*if (this->batterylevel_sensor_ != nullptr) {
      // To be fixed
      // This is not giving the right value - mostly there to have some sample sensor...
      float vbat = GetBatVoltage();
      float batterylevel = 100.0 * ((vbat - 3.0) / (4.1 - 3.0));

      ESP_LOGD(TAG, "Got Battery Level=%f (%f)", batterylevel, vbat);
      if (batterylevel > 100.) {
        batterylevel = 100;
      }
      this->batterylevel_sensor_->publish_state(batterylevel);
    }
  
    if (this->charging_sensor_ != nullptr) {
      bool battery_state = GetBatState();
      bool charging = GetChargingState();
      ESP_LOGD(TAG, "Got Charging=%d state=%d", charging, battery_state);
      this->charging_sensor_->publish_state(charging);
    }*/
}

void AXP192Component::loop() {
  // IRQ triggers
#ifdef USE_BINARY_SENSORS
  this->do_irqs_();
#endif
  UpdateBrightness();
}

void AXP192Component::power_off() {}

void AXP192Component::prepare_sleep() {}

bool AXP192Component::configure_axp() {
  for (auto &reg : this->registers_) {
    if (!save_register(reg.first)) {
      return false;
    }
  }
  return true;
}

// NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
bool AXP192Component::update_register(RegisterLocations reg, uint8_t value, uint8_t clear_mask) {
  auto location = this->registers_.find(reg);
  if (location != this->registers_.end()) {
    location->second &= clear_mask;
    location->second |= value;
  }
  return location != this->registers_.end();
}

bool AXP192Component::load_register(RegisterLocations reg) {
  auto location = this->registers_.find(reg);
  if (location == this->registers_.end()) {
    return false;
  }
  auto contents = this->read_byte(detail::to_int(reg));
  if (contents.has_value()) {
    location->second = contents.value();
    ESP_LOGVV(this->get_component_source(), "Read %s from 0x%02X", detail::format_bits(location->second).c_str(),
              detail::to_int(reg));
  }
  return contents.has_value();
}

bool AXP192Component::save_register(RegisterLocations reg) {
  auto location = this->registers_.find(reg);
  if (location == this->registers_.end()) {
    return false;
  }

  auto base = 0xFF;
  auto mask = this->register_masks_.find(reg);
  if (mask != this->register_masks_.end()) {
    base = this->read_byte(detail::to_int(reg)).value_or(0x00) & mask->second;
  }

  base |= location->second;
  if (this->write_byte(detail::to_int(reg), base)) {
    ESP_LOGVV(this->get_component_source(), "Wrote %s to 0x%02X", detail::format_bits(base).c_str(),
              detail::to_int(reg));
    return true;
  }
  return false;
}

void AXP192Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP192:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(this->get_component_source(), "Registers:");
  for(auto reg : this->registers_) {
    ESP_LOGCONFIG(this->get_component_source(), "  %s %s", detail::to_hex(reg.first).c_str(),
                  detail::format_bits(reg.second).c_str());
  }
}

void AXP192Component::debug_log_register_(RegisterLocations reg) {
  auto val = this->read_byte(detail::to_int(reg));
  if (val.has_value()) {
    ESP_LOGD(this->get_component_source(), "Read %s from 0x%X", detail::format_bits(val.value()).c_str(),
             detail::to_int(reg));
  } else {
    ESP_LOGD(this->get_component_source(), "Failed to read 0x%X", detail::to_int(reg));
  }
}

bool AXP192Component::configure_ldoio0(bool enable) {
#ifdef USE_SWITCH
  auto location = this->power_control_.find(OutputPin::OUTPUT_LDOIO0);
  this->load_register(RegisterLocations::GPIO_CONTROL);
  this->set_ldoio0_mode(enable ? LDOio0Control::LOWNOISE_LDO : LDOio0Control::FLOATING);
  if (this->save_register(RegisterLocations::GPIO_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_LDOIO0, this->get_ldoio0_enabled());
    return true;
  }
#endif
  return false;
}

void AXP192Component::set_voff(VoffVoltage voff) {
  this->update_register(RegisterLocations::VOFF_VOLTAGE, detail::to_int(voff), 0b11111000);
}

void AXP192Component::set_charge_voltage(ChargeVoltage voltage) {
  this->update_register(RegisterLocations::CHARGE_CONTROL_REG1, detail::to_int(voltage), 0b10011111);
}

void AXP192Component::set_charge_current(ChargeCurrent current) {
  this->update_register(RegisterLocations::CHARGE_CONTROL_REG1, detail::to_int(current), 0b11110000);
}

void AXP192Component::set_vbus_ipsout(VBusIpsout val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b01111111);
}

void AXP192Component::set_vbus_hold_current_limited(VBusHoldCurrentLimited val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b11111101);
}

void AXP192Component::set_vbus_hold_current_limit(VBusHoldCurrentLimit val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b11111110);
}

void AXP192Component::set_vbus_hold_voltage_limited(VBusHoldVoltageLimited val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b10111111);
}

void AXP192Component::set_vbus_hold_voltage_limit(VBusHoldVoltageLimit val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b11000111);
}

void AXP192Component::set_disable_rtc(bool disable_rtc) {
  this->update_register(RegisterLocations::BATTERY_BACKUP_CONTROL, disable_rtc ? 0x0 : 0b10000000, 0b11000111);
}

void AXP192Component::set_disable_ldo2(bool disable_ldo2) {
  this->update_register(RegisterLocations::DCDC13_LDO23_CONTROL, disable_ldo2 ? 0x0 : 0b00000100, 0b11111011);
}

void AXP192Component::set_disable_ldo3(bool disable_ldo3) {
  this->update_register(RegisterLocations::DCDC13_LDO23_CONTROL, disable_ldo3 ? 0x0 : 0b11110111, 0b00001000);
}

void AXP192Component::set_disable_dcdc1(bool disable_dcdc1) {
  this->update_register(RegisterLocations::DCDC13_LDO23_CONTROL, disable_dcdc1 ? 0x0 : 0b00000010, 0b11111101);
}

void AXP192Component::set_disable_dcdc3(bool disable_dcdc3) {
  this->update_register(RegisterLocations::DCDC13_LDO23_CONTROL, disable_dcdc3 ? 0x0 : 0b00000001, 0b11111110);
}

void AXP192Component::set_dcdc1_voltage(uint32_t dcdc1_voltage) {
  this->update_register(RegisterLocations::DCDC1_VOLTAGE,
                        detail::constrained_remap<uint32_t, 700, 3500, 0x0, 0x7F>(dcdc1_voltage), 0b10000000);
}

void AXP192Component::set_dcdc3_voltage(uint32_t dcdc3_voltage) {
  this->update_register(RegisterLocations::DCDC3_VOLTAGE,
                        detail::constrained_remap<uint32_t, 700, 3500, 0x0, 0x7F>(dcdc3_voltage), 0b10000000);
}

void AXP192Component::set_ldo2_voltage(uint32_t ldo2_voltage) {
  this->update_register(RegisterLocations::LDO23_VOLTAGE,
                        (detail::constrained_remap<int, 1800, 3300, 0x0, 0x0F>(ldo2_voltage) << 4), 0b00001111);
}

void AXP192Component::set_ldo3_voltage(uint32_t ldo3_voltage) {
  this->update_register(RegisterLocations::LDO23_VOLTAGE,
                        detail::constrained_remap<uint32_t, 1800, 3300, 0x0, 0x0F>(ldo3_voltage), 0b11110000);
}

void AXP192Component::set_ldoio0_voltage(uint32_t ldoio0_voltage) {
  this->update_register(RegisterLocations::GPIO_LDO_VOLTAGE,
                        (detail::constrained_remap<uint32_t, 1800, 3300, 0x0, 0x0F>(ldoio0_voltage) << 4), 0b00001111);
}

void AXP192Component::set_ldoio0_mode(LDOio0Control mode) {
  this->update_register(RegisterLocations::GPIO_CONTROL, detail::to_int(mode), 0b11111000);
}

bool AXP192Component::configure_ldo2(bool enable) {
  this->debug_log_register_(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->load_register(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->set_disable_ldo2(!enable);
  if (this->save_register(RegisterLocations::DCDC13_LDO23_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_LDO2, this->get_ldo2_enabled());
    return true;
  }
  return false;
}

bool AXP192Component::configure_ldo3(bool enable) {
  this->load_register(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->set_disable_ldo3(!enable);
  if (this->save_register(RegisterLocations::DCDC13_LDO23_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_LDO3, this->get_ldo3_enabled());
    return true;
  }
  return false;
}

bool AXP192Component::configure_dcdc1(bool enable) {
  this->load_register(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->set_disable_dcdc1(!enable);
  if (this->save_register(RegisterLocations::DCDC13_LDO23_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_DCDC1, this->get_dcdc1_enabled());
    return true;
  }
  return false;
}

bool AXP192Component::configure_dcdc3(bool enable) {
  this->load_register(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->set_disable_dcdc3(!enable);
  if (this->save_register(RegisterLocations::DCDC13_LDO23_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_DCDC3, this->get_dcdc3_enabled());
    return true;
  }
  return false;
}

bool AXP192Component::configure_ldo2_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 1800, 3300);
  this->load_register(RegisterLocations::LDO23_VOLTAGE);
  this->set_ldo2_voltage(scaled);
  return this->save_register(RegisterLocations::LDO23_VOLTAGE);
}

bool AXP192Component::configure_ldo3_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 1800, 3300);
  this->load_register(RegisterLocations::LDO23_VOLTAGE);
  this->set_ldo3_voltage(scaled);
  return this->save_register(RegisterLocations::LDO23_VOLTAGE);
}

bool AXP192Component::configure_dcdc1_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 700, 3500);
  this->load_register(RegisterLocations::DCDC1_VOLTAGE);
  this->set_dcdc1_voltage(scaled);
  return this->save_register(RegisterLocations::DCDC1_VOLTAGE);
}

bool AXP192Component::configure_dcdc3_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 700, 3500);
  this->load_register(RegisterLocations::DCDC3_VOLTAGE);
  this->set_dcdc3_voltage(scaled);
  return this->save_register(RegisterLocations::DCDC3_VOLTAGE);
}

bool AXP192Component::configure_ldoio0_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 1800, 3300);
  this->load_register(RegisterLocations::GPIO_LDO_VOLTAGE);
  this->set_ldoio0_voltage(scaled);
  return this->save_register(RegisterLocations::GPIO_LDO_VOLTAGE);
}

bool AXP192Component::get_ldo2_enabled() {
  return (this->registers_.at(RegisterLocations::DCDC13_LDO23_CONTROL) & 0b00000100) != 0;
}

bool AXP192Component::get_ldo3_enabled() {
  return (this->registers_.at(RegisterLocations::DCDC13_LDO23_CONTROL) & 0b00001000) != 0;
}

bool AXP192Component::get_dcdc1_enabled() {
  return (this->registers_.at(RegisterLocations::DCDC13_LDO23_CONTROL) & 0b00000001) != 0;
}

bool AXP192Component::get_dcdc3_enabled() {
  return (this->registers_.at(RegisterLocations::DCDC13_LDO23_CONTROL) & 0b00000010) != 0;
}

bool AXP192Component::get_ldoio0_enabled() {
  return (this->registers_.at(RegisterLocations::GPIO_CONTROL) & 0b00000111) == 0b00000010;
}

float AXP192Component::get_ldo2_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::LDO23_VOLTAGE) & 0b11110000) >> 4;
  return remap<float, uint8_t>(raw, 0, 0x0F, 0, 100);
}

float AXP192Component::get_ldo3_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::LDO23_VOLTAGE) & 0b00001111);
  return remap<float, uint8_t>(raw, 0, 0x0F, 0, 100);
}

float AXP192Component::get_dcdc1_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::DCDC1_VOLTAGE) & 0b01111111);
  return remap<float, uint8_t>(raw, 0, 0x7F, 0, 100);
}

float AXP192Component::get_dcdc3_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::DCDC3_VOLTAGE) & 0b01111111);
  return remap<float, uint8_t>(raw, 0, 0x7F, 0, 100);
}

float AXP192Component::get_ldoio0_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::GPIO_LDO_VOLTAGE) & 0b11110000) >> 4;
  return remap<float, uint8_t>(raw, 0, 0x0F, 0, 100);
}

void AXP192Component::set_dcdc2_voltage(uint32_t dcdc2_voltage) {
  this->update_register(RegisterLocations::DCDC2_VOLTAGE,
                        detail::constrained_remap<uint32_t, 700, 2275, 0x0, 0x3F>(dcdc2_voltage), 0b11000000);
}

void AXP192Component::set_disable_dcdc2(bool disable_dcdc2) {
  this->update_register(RegisterLocations::EXTEN_DCDC2_CONTROL, disable_dcdc2 ? 0x0 : 0b00000101, 0b11111010);
}

bool AXP192Component::configure_dcdc2(bool enable) {
  this->load_register(RegisterLocations::EXTEN_DCDC2_CONTROL);
  this->set_disable_dcdc2(!enable);
  return this->save_register(RegisterLocations::EXTEN_DCDC2_CONTROL);
}

float AXP192Component::get_setup_priority() const { return setup_priority::DATA; }

#ifdef USE_BINARY_SENSOR
void AXP192Component::do_irqs_() {
  // Read all IRQ registers at once
  auto buffer = this->read_bytes<4>(detail::to_int(RegisterLocations::IRQ_STATUS_REGISTER1));
  if (buffer.has_value()) {
    auto bits = encode_uint32(buffer.value()[0], buffer.value()[1], buffer.value()[2], buffer.value()[3]);
    if (bits != this->last_irq_buffer_) {
      ESP_LOGV(this->get_component_source(),
               "IRQ Register Load: IRQ_STATUS_REGISTER1: 0x%08X, raw: 0x%02X, 0x%02X, 0x%02X, 0x%02X", bits,
               buffer.value()[0], buffer.value()[1], buffer.value()[2], buffer.value()[3]);
      this->last_irq_buffer_ = bits;
      uint32_t clear_bits = 0x00;
      for (auto irq : this->irqs_) {
        auto val = (detail::to_int(irq.first) & bits) != 0;
        this->publish_helper_(irq.first, val);
        if (val) {
          ESP_LOGV(this->get_component_source(), "Accept: %s: 0x%08X, clear bits: 0x%08X",
                   detail::to_hex(irq.first).c_str(), detail::to_int(irq.first), clear_bits);
          clear_bits |= detail::to_int(irq.first);
        }
      }
      auto output = decode_value(clear_bits);
      if (output[0] > 0) {
        this->write_byte(detail::to_int(RegisterLocations::IRQ_STATUS_REGISTER1), output[0]);
      }
      if (output[1] > 0) {
        this->write_byte(detail::to_int(RegisterLocations::IRQ_STATUS_REGISTER2), output[1]);
      }
      if (output[2] > 0) {
        this->write_byte(detail::to_int(RegisterLocations::IRQ_STATUS_REGISTER3), output[2]);
      }
      if (output[3] > 0) {
        this->write_byte(detail::to_int(RegisterLocations::IRQ_STATUS_REGISTER4), output[3]);
      }
      ESP_LOGV(this->get_component_source(),
               "IRQ Register Save: IRQ_STATUS_REGISTER1: 0x%08X, raw: 0x%02X, 0x%02X, 0x%02X, 0x%02X", clear_bits,
               output[0], output[1], output[2], output[3]);
    }
  }
}

void AXP192Component::enable_irq(IrqType irq) {

  ESP_LOGV(this->get_component_source(), "Enable IRQ %s bits: 0x%80X", detail::to_hex(irq).c_str(),
           detail::to_int(irq));
  if (detail::to_int(irq) > 0x800000) {
    ESP_LOGV(this->get_component_source(), "IRQ Register Save: IRQ_ENABLE_REGISTER1 value: 0x%02X, mask: 0x%02X",
             (detail::to_int(irq) >> 24) & 0xFF, ~(detail::to_int(irq) >> 24) & 0xFF);
    load_register(RegisterLocations::IRQ_ENABLE_REGISTER1);
    update_register(RegisterLocations::IRQ_ENABLE_REGISTER1, (detail::to_int(irq) >> 24) & 0xFF,
                    ~(detail::to_int(irq) >> 24) & 0xFF);
    save_register(RegisterLocations::IRQ_ENABLE_REGISTER1);
    return;
  }
  if (detail::to_int(irq) > 0x8000) {
    ESP_LOGV(this->get_component_source(), "IRQ Register Save: IRQ_ENABLE_REGISTER2 value: 0%02X, mask: 0%02X",
             (detail::to_int(irq) >> 16) & 0xFF, ~(detail::to_int(irq) >> 16) & 0xFF);
    load_register(RegisterLocations::IRQ_ENABLE_REGISTER2);
    update_register(RegisterLocations::IRQ_ENABLE_REGISTER2, (detail::to_int(irq) >> 16) & 0xFF,
                    ~(detail::to_int(irq) >> 16) & 0xFF);
    save_register(RegisterLocations::IRQ_ENABLE_REGISTER2);
    return;
  }
  if (detail::to_int(irq) > 0x80) {
    ESP_LOGV(this->get_component_source(), "IRQ Register Save: IRQ_ENABLE_REGISTER3 value: 0%02X, mask: 0%02X",
             (detail::to_int(irq) >> 8) & 0xFF, ~(detail::to_int(irq) >> 8) & 0xFF);
    load_register(RegisterLocations::IRQ_ENABLE_REGISTER3);
    update_register(RegisterLocations::IRQ_ENABLE_REGISTER3, (detail::to_int(irq) >> 8) & 0xFF,
                    ~(detail::to_int(irq) >> 8) & 0xFF);
    save_register(RegisterLocations::IRQ_ENABLE_REGISTER3);
    return;
  }
  ESP_LOGV(this->get_component_source(), "IRQ Register Save: IRQ_ENABLE_REGISTER4 value: 0%02X, mask: 0%02X",
           detail::to_int(irq) & 0xFF, ~(detail::to_int(irq) & 0xFF));
  load_register(RegisterLocations::IRQ_ENABLE_REGISTER4);
  update_register(RegisterLocations::IRQ_ENABLE_REGISTER4, detail::to_int(irq) & 0xFF, ~(detail::to_int(irq)) & 0xFF);
  save_register(RegisterLocations::IRQ_ENABLE_REGISTER4);
}
#endif

void AXP192Component::update_powercontrol(OutputPin pin, bool value) {
#ifdef USE_SWITCH
  if (this->power_control_.empty())
    return;

  auto location = this->power_control_.find(pin);
  if (location != this->power_control_.end()) {
    location->second->publish_state(value);
  }
#endif
}

void AXP192Component::publish_helper_(SensorType type, float state) {
#ifdef USE_SENSOR
  auto sensor = this->sensors_.find(type);
  if (sensor != this->sensors_.end()) {
    sensor->second->publish_state(state);
  }
#endif
}

void AXP192Component::publish_helper_(IrqType type, bool state) {
#ifdef USE_BINARY_SENSOR
  auto sensor = this->irqs_.find(type);
  if (sensor != this->irqs_.end()) {
    sensor->second->publish_state(state);
  }
#endif
}

void AXP192Component::publish_helper_(MonitorType type, bool state) {
#ifdef USE_BINARY_SENSOR
  auto sensor = this->monitors_.find(type);
  if (sensor != this->monitors_.end()) {
    sensor->second->publish_state(state);
  }
#endif
}

void AXP192Component::register_irq(IrqType type, AXP192BinarySensor *irq) {
#ifdef USE_BINARY_SENSOR
  irqs_.insert(std::make_pair(type, irq));
#endif
}

void AXP192Component::register_sensor(SensorType type, AXP192Sensor *sensor) {
#ifdef USE_SENSOR
  sensors_.insert(std::make_pair(type, sensor));
#endif
}

void AXP192Component::register_output(OutputPin pin, AXP192Output *output) {
#ifdef USE_OUTPUT
  output_control_.insert(std::make_pair(pin, output));
#endif
}

void AXP192Component::register_switch(OutputPin pin, AXP192Switch *output) {
#ifdef USE_SWITCH
  power_control_.insert(std::make_pair(pin, output));
#endif
}

void AXP192Component::register_monitor(MonitorType type, AXP192BinarySensor *monitor) {
#ifdef USE_BINARY_SENSOR
  monitors_.insert(std::make_pair(type, monitor));
#endif
}


void AXP192Component::begin(bool disableLDO2, bool disableLDO3, bool disableRTC, bool disableDCDC1, bool disableDCDC3)
{
  switch (this->model_) {
    case AXP192Model::M5STICKC:
    {
        // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
        Write1Byte(0x28, 0xcc);
        break;
    }
    case AXP192Model::M5CORE2:
    {
        // Set DCDC3 (TFT_LED & TFT) 3.0V
        Write1Byte(0x27, 0xcc);
        // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
        Write1Byte(0x28, 0xcc);
        break;
    }
    case AXP192Model::M5TOUGH:
    {
        // Set DCDC3 (TFT_LED & TFT) 3.0V
        Write1Byte(0x27, 0xcc);
        // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
        Write1Byte(0x28, 0xcc);
        break;
    }
  }

    // Set ADC sample rate to 200hz
    Write1Byte(0x84, 0b11110010);

    // Set ADC to All Enable
    Write1Byte(0x82, 0xff);

    // Bat charge voltage to 4.2, Current 100MA
    Write1Byte(0x33, 0xc0);

    // Depending on configuration enable LDO2, LDO3, DCDC1, DCDC3.
    uint8_t buf = (Read8bit(0x12) & 0xef) | 0x4D;
    if(disableLDO3) buf &= ~(1<<3);
    if(disableLDO2) buf &= ~(1<<2);
    if(disableDCDC3) buf &= ~(1<<1);
    if(disableDCDC1) buf &= ~(1<<0);
    Write1Byte(0x12, buf);

    // 128ms power on, 4s power off
    Write1Byte(0x36, 0x0C);

    if(!disableRTC)
    {
        // Set RTC voltage to 3.3V
        Write1Byte(0x91, 0xF0);

        // Set GPIO0 to LDO
        Write1Byte(0x90, 0x02);
    }

    // Disable vbus hold limit
    Write1Byte(0x30, 0x80);

    // Set temperature protection
    Write1Byte(0x39, 0xfc);

    // Enable RTC BAT charge
    Write1Byte(0x35, 0xa2 & (disableRTC ? 0x7F : 0xFF));

    // Enable bat detection
    Write1Byte(0x32, 0x46);

}

void AXP192Component::Write1Byte( uint8_t Addr ,  uint8_t Data )
{
    this->write_byte(Addr, Data);
}

uint8_t AXP192Component::Read8bit( uint8_t Addr )
{
    uint8_t data;
    this->read_byte(Addr, &data);
    return data;
}

uint16_t AXP192Component::Read12Bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr,2,buf);
    Data = ((buf[0] << 4) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read13Bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr,2,buf);
    Data = ((buf[0] << 5) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read16bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[2];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read24bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[3];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read32bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[4];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

void AXP192Component::ReadBuff( uint8_t Addr , uint8_t Size , uint8_t *Buff )
{
    this->read_bytes(Addr, Buff, Size);
}

void AXP192Component::UpdateBrightness()
{
    if (brightness_ == curr_brightness_)
    {
        return;
    }

    float tempBrightness = brightness_;
    //ESP_LOGD(TAG, "Brightness=%f (Curr: %f)", brightness_, curr_brightness_);
    

    const uint8_t c_min = 7;
    const uint8_t c_max = 12;
    auto ubri = c_min + static_cast<uint8_t>(tempBrightness * (c_max - c_min));

    if (ubri > c_max)
    {
        ubri = c_max;
    }
    switch (this->model_) {
      case AXP192Model::M5STICKC:
        auto tempbyte this->read_byte(detail::to_int(RegisterLocations::LDO23_VOLTAGE)).value_and(0x0f);
        tempbyte |= (ubri << 4);
        this->write_byte(detail::to_int(RegisterLocations::LDO23_VOLTAGE), tempbyte);
        break;
      case AXP192Model::M5CORE2:
      case AXP192Model::M5TOUGH:
        auto tempbyte this->read_byte(detail::to_int(RegisterLocations::DCDC3_VOLTAGE)).value_and(0x80);
        tempbyte |= (ubri << 3);
        this->write_byte(detail::to_int(RegisterLocations::DCDC3_VOLTAGE), tempbyte);
        break;
    }
    if (tempBrightness == 0) {
      SetLDO2(false);
    } else if(curr_brightness_ == 0) {
      SetLDO2(true);
    }
    curr_brightness_ = tempBrightness;
}

bool AXP192Component::GetBatState()
{
    if( Read8bit(0x01) | 0x20 )
        return true;
    else
        return false;
}

bool AXP192Component::GetChargingState()
{
    // reading 0x00 bit 6
    if( (Read8bit(0x01) >> 5) & 0x01 )
        return true;
    else
        return false;
}

uint8_t AXP192Component::GetBatData()
{
    return Read8bit(0x75);
}
//---------coulombcounter_from_here---------
//enable: void EnableCoulombcounter(void);
//disable: void DisableCOulombcounter(void);
//stop: void StopCoulombcounter(void);
//clear: void ClearCoulombcounter(void);
//get charge data: uint32_t GetCoulombchargeData(void);
//get discharge data: uint32_t GetCoulombdischargeData(void);
//get coulomb val affter calculation: float GetCoulombData(void);
//------------------------------------------
void  AXP192Component::EnableCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0x80 );
}

void  AXP192Component::DisableCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0x00 );
}

void  AXP192Component::StopCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0xC0 );
}

void  AXP192Component::ClearCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0xA0 );
}

uint32_t AXP192Component::GetCoulombchargeData(void)
{
    return Read32bit(0xB0);
}

uint32_t AXP192Component::GetCoulombdischargeData(void)
{
    return Read32bit(0xB4);
}

float AXP192Component::GetCoulombData(void)
{

  uint32_t coin = 0;
  uint32_t coout = 0;

  coin = GetCoulombchargeData();
  coout = GetCoulombdischargeData();

  //c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
  //Adc rate can be read from 84H ,change this variable if you change the ADC reate
  float ccc = 65536 * 0.5 * (coin - coout) / 3600.0 / 25.0;
  return ccc;

}
//----------coulomb_end_at_here----------

uint16_t AXP192Component::GetVbatData(void){

    uint16_t vbat = 0;
    uint8_t buf[2];
    ReadBuff(0x78,2,buf);
    vbat = ((buf[0] << 4) + buf[1]); // V
    return vbat;
}

uint16_t AXP192Component::GetVinData(void)
{
    uint16_t vin = 0;
    uint8_t buf[2];
    ReadBuff(0x56,2,buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP192Component::GetIinData(void)
{
    uint16_t iin = 0;
    uint8_t buf[2];
    ReadBuff(0x58,2,buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP192Component::GetVusbinData(void)
{
    uint16_t vin = 0;
    uint8_t buf[2];
    ReadBuff(0x5a,2,buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP192Component::GetIusbinData(void)
{
    uint16_t iin = 0;
    uint8_t buf[2];
    ReadBuff(0x5C,2,buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP192Component::GetIchargeData(void)
{
    uint16_t icharge = 0;
    uint8_t buf[2];
    ReadBuff(0x7A,2,buf);
    icharge = ( buf[0] << 5 ) + buf[1] ;
    return icharge;
}

uint16_t AXP192Component::GetIdischargeData(void)
{
    uint16_t idischarge = 0;
    uint8_t buf[2];
    ReadBuff(0x7C,2,buf);
    idischarge = ( buf[0] << 5 ) + buf[1] ;
    return idischarge;
}

uint16_t AXP192Component::GetTempData(void)
{
    uint16_t temp = 0;
    uint8_t buf[2];
    ReadBuff(0x5e,2,buf);
    temp = ((buf[0] << 4) + buf[1]);
    return temp;
}

uint32_t AXP192Component::GetPowerbatData(void)
{
    uint32_t power = 0;
    uint8_t buf[3];
    ReadBuff(0x70,2,buf);
    power = (buf[0] << 16) + (buf[1] << 8) + buf[2];
    return power;
}

uint16_t AXP192Component::GetVapsData(void)
{
    uint16_t vaps = 0;
    uint8_t buf[2];
    ReadBuff(0x7e,2,buf);
    vaps = ((buf[0] << 4) + buf[1]);
    return vaps;
}

void AXP192Component::SetSleep(void)
{
    Write1Byte(0x31 , Read8bit(0x31) | ( 1 << 3)); // Power off voltag 3.0v
    Write1Byte(0x90 , Read8bit(0x90) | 0x07); // GPIO1 floating
    Write1Byte(0x82, 0x00); // Disable ADCs
    Write1Byte(0x12, Read8bit(0x12) & 0xA1); // Disable all outputs but DCDC1
}

// -- sleep
void AXP192Component::DeepSleep(uint64_t time_in_us)
{
    SetSleep();
    //esp_sleep_enable_ext0_wakeup((gpio_num_t)37, 0 /* LOW */);
    /*if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);*/
}

void AXP192Component::LightSleep(uint64_t time_in_us)
{
    /*if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    esp_light_sleep_start();*/
}

// 0 not press, 0x01 long press, 0x02 press
uint8_t AXP192Component::GetBtnPress()
{
    uint8_t state = Read8bit(0x46);
    if(state)
    {
        Write1Byte( 0x46 , 0x03 );
    }
    return state;
}

uint8_t AXP192Component::GetWarningLevel(void)
{
    return Read8bit(0x47) & 0x01;
}

float AXP192Component::GetBatVoltage()
{
    float ADCLSB = 1.1 / 1000.0;
    uint16_t ReData = Read12Bit( 0x78 );
    return ReData * ADCLSB;
}

float AXP192Component::GetBatCurrent()
{
    float ADCLSB = 0.5;
    uint16_t CurrentIn = Read13Bit( 0x7A );
    uint16_t CurrentOut = Read13Bit( 0x7C );
    return ( CurrentIn - CurrentOut ) * ADCLSB;
}

float AXP192Component::GetVinVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit( 0x56 );
    return ReData * ADCLSB;
}

float AXP192Component::GetVinCurrent()
{
    float ADCLSB = 0.625;
    uint16_t ReData = Read12Bit( 0x58 );
    return ReData * ADCLSB;
}

float AXP192Component::GetVBusVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit( 0x5A );
    return ReData * ADCLSB;
}

float AXP192Component::GetVBusCurrent()
{
    float ADCLSB = 0.375;
    uint16_t ReData = Read12Bit( 0x5C );
    return ReData * ADCLSB;
}

float AXP192Component::GetTempInAXP192()
{
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData = Read12Bit( 0x5E );
    return OFFSET_DEG_C + ReData * ADCLSB;
}

float AXP192Component::GetBatPower()
{
    float VoltageLSB = 1.1;
    float CurrentLCS = 0.5;
    uint32_t ReData = Read24bit( 0x70 );
    return  VoltageLSB * CurrentLCS * ReData/ 1000.0;
}

float AXP192Component::GetBatChargeCurrent()
{
    float ADCLSB = 0.5;
    uint16_t ReData = Read13Bit( 0x7A );
    return ReData * ADCLSB;
}

float AXP192Component::GetAPSVoltage()
{
    float ADCLSB = 1.4  / 1000.0;
    uint16_t ReData = Read12Bit( 0x7E );
    return ReData * ADCLSB;
}

float AXP192Component::GetBatCoulombInput()
{
    uint32_t ReData = Read32bit( 0xB0 );
    return ReData * 65536 * 0.5 / 3600 /25.0;
}

float AXP192Component::GetBatCoulombOut()
{
    uint32_t ReData = Read32bit( 0xB4 );
    return ReData * 65536 * 0.5 / 3600 /25.0;
}

void AXP192Component::SetCoulombClear()
{
    Write1Byte(0xB8,0x20);
}

void AXP192Component::SetLDO2( bool State )
{
    uint8_t buf = Read8bit(0x12);
    if( State == true )
    {
        buf = (1<<2) | buf;
    }
    else
    {
        buf = ~(1<<2) & buf;
    }
    Write1Byte( 0x12 , buf );
}

void AXP192Component::SetLDO3(bool State)
{
    uint8_t buf = Read8bit(0x12);
    if( State == true )
    {
        buf = (1<<3) | buf;
    }
    else
    {
        buf = ~(1<<3) & buf;
    }
    Write1Byte( 0x12 , buf );
}

void AXP192Component::SetChargeCurrent(uint8_t current)
{
    uint8_t buf = Read8bit(0x33);
    buf = (buf & 0xf0) | (current & 0x07);
    Write1Byte(0x33, buf);
}

void AXP192Component::PowerOff()
{
    Write1Byte(0x32, Read8bit(0x32) | 0x80);
}

void AXP192Component::SetAdcState(bool state)
{
    Write1Byte(0x82, state ? 0xff : 0x00);
}

std::string AXP192Component::GetStartupReason() {
  /*esp_reset_reason_t reset_reason = ::esp_reset_reason();
  if (reset_reason == ESP_RST_DEEPSLEEP) {
    esp_sleep_source_t wake_reason = esp_sleep_get_wakeup_cause();
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT0)
      return "ESP_SLEEP_WAKEUP_EXT0";
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT0)
      return "ESP_SLEEP_WAKEUP_EXT0";
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT1)
      return "ESP_SLEEP_WAKEUP_EXT1";
    if (wake_reason == ESP_SLEEP_WAKEUP_TIMER)
      return "ESP_SLEEP_WAKEUP_TIMER";
    if (wake_reason == ESP_SLEEP_WAKEUP_TOUCHPAD)
      return "ESP_SLEEP_WAKEUP_TOUCHPAD";
    if (wake_reason == ESP_SLEEP_WAKEUP_ULP)
      return "ESP_SLEEP_WAKEUP_ULP";
    if (wake_reason == ESP_SLEEP_WAKEUP_GPIO)
      return "ESP_SLEEP_WAKEUP_GPIO";
    if (wake_reason == ESP_SLEEP_WAKEUP_UART)
      return "ESP_SLEEP_WAKEUP_UART";
    return std::string{"WAKEUP_UNKNOWN_REASON"};
  }

  if (reset_reason == ESP_RST_UNKNOWN)
    return "ESP_RST_UNKNOWN";
  if (reset_reason == ESP_RST_POWERON)
    return "ESP_RST_POWERON";
  if (reset_reason == ESP_RST_SW)
    return "ESP_RST_SW";
  if (reset_reason == ESP_RST_PANIC)
    return "ESP_RST_PANIC";
  if (reset_reason == ESP_RST_INT_WDT)
    return "ESP_RST_INT_WDT";
  if (reset_reason == ESP_RST_TASK_WDT)
    return "ESP_RST_TASK_WDT";
  if (reset_reason == ESP_RST_WDT)
    return "ESP_RST_WDT";
  if (reset_reason == ESP_RST_BROWNOUT)
    return "ESP_RST_BROWNOUT";
  if (reset_reason == ESP_RST_SDIO)
    return "ESP_RST_SDIO";*/
  return std::string{"RESET_UNKNOWN_REASON"};
}

}
}

