#pragma once
#include "esphome/core/automation.h"
#include "nextion_sensor.h"

namespace esphome {
namespace nextion {

template<typename... Ts> class NextionSensorPublishAction : public Action<Ts...> {
 public:
  NextionSensorPublishAction(NextionSensor *sensor) : sensor_(sensor) {}
  TEMPLATABLE_VALUE(float, state)
  TEMPLATABLE_VALUE(bool, publish)
  TEMPLATABLE_VALUE(bool, send_to_nextion)

  void set_state(Ts... x) override { this->sensor_->set_state(this->state_.value(x...), this->publish_.optional_value(x...), this->send_to_nextion_.optional_value(x...)); }

 protected:
  NextionSensor *sensor_;
};

}
}