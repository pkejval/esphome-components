#pragma once

#include "esphome/core/automation.h"

#include "pc_fan_controller.h"

namespace esphome {
namespace pc_fan_controller {

template<typename... Ts> class SetTemperatureAction : public Action<Ts...> {
 public:
  explicit SetTemperatureAction(PcFanController *controller) : controller_(controller) {}

  void set_source(const char *source) { this->source_ = source; }
  TEMPLATABLE_VALUE(float, value)

  void play(Ts... x) override { this->controller_->set_temperature(this->source_, this->value_.value(x...)); }

 protected:
  PcFanController *controller_;
  const char *source_{nullptr};
};

}  // namespace pc_fan_controller
}  // namespace esphome
