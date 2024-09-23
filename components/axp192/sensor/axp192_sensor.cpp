#include "axp192_sensor.h"
#include "esphome/core/log.h"

#define TAG this->get_component_source()

namespace esphome {
namespace axp192 {

void AXP192Sensor::dump_config() { LOG_SENSOR("Monitor: ", detail::to_hex(this->sensor_.value()).c_str(), this); }
void AXP192Sensor::setup() { this->get_parent()->register_sensor(sensor_.value(), this); }
float AXP192Sensor::get_setup_priority() const { return setup_priority::DATA; };
void AXP192Sensor::set_sensor(SensorType type) { sensor_ = type; };

}  // namespace axp192
}  // namespace esphome
