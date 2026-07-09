#include "pc_fan_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::pc_fan_controller {

static const char *const TAG = "pc_fan_controller";

void PcFanController::setup() {
  this->pref_ = global_preferences->make_preference<StoredConfig>(PREFERENCE_MAGIC, true);
  this->load_config_();
  this->setup_ledc_();
  this->regulate_();

  if (web_server_base::global_web_server_base != nullptr) {
    web_server_base::global_web_server_base->add_handler(this);
    web_server_base::global_web_server_base->init();
  }
}

void PcFanController::loop() {
  const uint32_t now = millis();
  if ((uint32_t) (now - this->last_regulation_ms_) >= this->update_interval_ms_) {
    this->last_regulation_ms_ = now;
    this->regulate_();
  }
}

void PcFanController::dump_config() {
  ESP_LOGCONFIG(TAG, "PC Fan Controller:");
  ESP_LOGCONFIG(TAG, "  Channels: %u", this->channel_count_);
  ESP_LOGCONFIG(TAG, "  PWM frequency: %" PRIu32 " Hz", this->pwm_frequency_);
  ESP_LOGCONFIG(TAG, "  Update interval: %" PRIu32 " ms", this->update_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Data timeout: %" PRIu32 " ms", this->data_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  UI path: %s", this->ui_path_.c_str());
  ESP_LOGCONFIG(TAG, "  API path: %s", this->api_path_.c_str());

  for (uint8_t i = 0; i < this->channel_count_; i++) {
    const auto &channel = this->channels_[i];
    ESP_LOGCONFIG(TAG, "  Channel %u: %s, GPIO%u, LEDC %u", channel.id, channel.name, channel.pin->get_pin(),
                  channel.ledc_channel);
  }
}

float PcFanController::get_setup_priority() const { return setup_priority::WIFI - 2.0f; }

void PcFanController::add_channel(InternalGPIOPin *pin, const char *name, uint8_t ledc_channel, bool inverted,
                                  const char *source, float min_pwm, float max_pwm, float default_pwm,
                                  float manual_pwm, float hysteresis, const char *curve_json) {
  if (this->channel_count_ >= this->channels_.size()) {
    ESP_LOGW(TAG, "Maximum channel count reached, ignoring channel %s", name);
    return;
  }

  auto &channel = this->channels_[this->channel_count_];
  channel.pin = pin;
  channel.id = this->channel_count_ + 1;
  channel.ledc_channel = ledc_channel;
  channel.inverted = inverted;
  channel.source = source_from_string_(source);
  channel.mode = ChannelMode::AUTO;
  channel.min_pwm = clamp_(min_pwm, 0.0f, 100.0f);
  channel.max_pwm = clamp_(max_pwm, channel.min_pwm, 100.0f);
  channel.default_pwm = clamp_(default_pwm, 0.0f, 100.0f);
  channel.manual_pwm = clamp_(manual_pwm, 0.0f, 100.0f);
  channel.hysteresis = clamp_(hysteresis, 0.0f, 20.0f);
  copy_string_(channel.name, sizeof(channel.name), name);

  if (!this->parse_curve_json_(channel, curve_json)) {
    this->set_default_curve_(channel);
  }

  this->channel_count_++;
}

void PcFanController::setup_ledc_() {
#ifdef USE_ESP32
  ledc_timer_config_t timer_config = {};
  timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_config.timer_num = LEDC_TIMER_0;
  timer_config.duty_resolution = LEDC_TIMER_10_BIT;
  timer_config.freq_hz = this->pwm_frequency_;
  timer_config.clk_cfg = LEDC_AUTO_CLK;

  esp_err_t timer_result = ledc_timer_config(&timer_config);
  if (timer_result != ESP_OK) {
    ESP_LOGE(TAG, "LEDC timer setup failed: %d", timer_result);
    this->mark_failed();
    return;
  }

  for (uint8_t i = 0; i < this->channel_count_; i++) {
    auto &channel = this->channels_[i];

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = channel.pin->get_pin();
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.channel = static_cast<ledc_channel_t>(channel.ledc_channel);
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = LEDC_TIMER_0;
    channel_config.duty = this->duty_for_pwm_(channel, channel.default_pwm);
    channel_config.hpoint = 0;

    esp_err_t channel_result = ledc_channel_config(&channel_config);
    channel.setup_ok = channel_result == ESP_OK;

    if (!channel.setup_ok) {
      ESP_LOGE(TAG, "LEDC channel setup failed for %s: %d", channel.name, channel_result);
    }
  }
#endif
}

void PcFanController::regulate_() {
  for (uint8_t i = 0; i < this->channel_count_; i++) {
    this->regulate_channel_(this->channels_[i]);
  }
}

void PcFanController::regulate_channel_(Channel &channel) {
  auto publish_entities = [&]() {
    if (channel.temperature_sensor != nullptr) {
      channel.temperature_sensor->publish_state(channel.source_temp);
    }
    if (channel.pwm_sensor != nullptr) {
      channel.pwm_sensor->publish_state(channel.applied_pwm);
    }
    if (channel.status_text_sensor != nullptr) {
      channel.status_text_sensor->publish_state(channel.status);
    }
  };

  if (!channel.setup_ok) {
    channel.applied_pwm = 0.0f;
    channel.failsafe = true;
    channel.last_temp = NAN;
    copy_string_(channel.status, sizeof(channel.status), "ERROR");
    channel.source_temp = NAN;
    publish_entities();
    return;
  }

  if (channel.mode == ChannelMode::OFF) {
    channel.source_temp = NAN;
    channel.failsafe = false;
    channel.last_temp = NAN;
    this->set_channel_pwm_(channel, 0.0f);
    copy_string_(channel.status, sizeof(channel.status), "OFF");
    publish_entities();
    return;
  }

  if (channel.mode == ChannelMode::MANUAL) {
    channel.source_temp = NAN;
    channel.failsafe = false;
    channel.last_temp = NAN;
    const float pwm = clamp_(channel.manual_pwm, 0.0f, 100.0f);
    this->set_channel_pwm_(channel, pwm);
    copy_string_(channel.status, sizeof(channel.status), "MANUAL");
    publish_entities();
    return;
  }

  float temperature = NAN;
  const TemperatureState temperature_state = this->source_temperature_(channel.source, &temperature);

  if (temperature_state != TemperatureState::FRESH) {
    channel.source_temp = NAN;
    channel.last_temp = NAN;

    channel.failsafe = true;

    const float pwm = clamp_(channel.default_pwm, 0.0f, 100.0f);
    this->set_channel_pwm_(channel, pwm);
    copy_string_(channel.status, sizeof(channel.status), "AUTO");
    publish_entities();
    return;
  }

  channel.source_temp = temperature;
  channel.failsafe = false;

  float pwm = this->curve_pwm_(channel, temperature);
  pwm = clamp_(pwm, channel.min_pwm, channel.max_pwm);
  this->set_channel_pwm_(channel, pwm);

  copy_string_(channel.status, sizeof(channel.status), "AUTO");
  publish_entities();
}

void PcFanController::set_channel_pwm_(Channel &channel, float pwm) {
#ifdef USE_ESP32
  pwm = clamp_(pwm, 0.0f, 100.0f);
  const uint32_t duty = this->duty_for_pwm_(channel, pwm);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel.ledc_channel), duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel.ledc_channel));
  channel.applied_pwm = pwm;
#endif
}

uint32_t PcFanController::duty_for_pwm_(const Channel &channel, float pwm) const {
  pwm = clamp_(pwm, 0.0f, 100.0f);
  const float output_pwm = channel.inverted ? 100.0f - pwm : pwm;
  return static_cast<uint32_t>(std::lround(output_pwm * PWM_DUTY_MAX / 100.0f));
}

PcFanController::TemperatureState PcFanController::source_temperature_(InputSource source, float *temperature) const {
  if (temperature == nullptr) {
    return TemperatureState::MISSING;
  }

  auto read_single = [&](float value, uint32_t stamp) -> TemperatureState {
    if (stamp == 0 || std::isnan(value)) {
      return TemperatureState::MISSING;
    }

    if (!this->input_fresh_(stamp)) {
      return TemperatureState::STALE;
    }

    *temperature = value;
    return TemperatureState::FRESH;
  };

  if (source == InputSource::CPU) {
    return read_single(this->input_cpu_, this->last_cpu_update_ms_);
  }

  if (source == InputSource::GPU) {
    return read_single(this->input_gpu_, this->last_gpu_update_ms_);
  }

  if (source == InputSource::OTHER) {
    return read_single(this->input_other_, this->last_other_update_ms_);
  }

  bool any_seen = false;
  bool any_stale = false;
  float best = NAN;

  auto consider = [&](float value, uint32_t stamp) {
    if (stamp == 0 || std::isnan(value)) {
      return;
    }

    any_seen = true;

    if (this->input_fresh_(stamp)) {
      if (std::isnan(best) || value > best) {
        best = value;
      }
    } else {
      any_stale = true;
    }
  };

  consider(this->input_cpu_, this->last_cpu_update_ms_);
  consider(this->input_gpu_, this->last_gpu_update_ms_);
  consider(this->input_other_, this->last_other_update_ms_);

  if (!std::isnan(best)) {
    *temperature = best;
    return TemperatureState::FRESH;
  }

  if (any_seen && any_stale) {
    return TemperatureState::STALE;
  }

  return TemperatureState::MISSING;
}

bool PcFanController::input_fresh_(uint32_t stamp) const {
  if (stamp == 0) {
    return false;
  }

  return (uint32_t) (millis() - stamp) <= this->data_timeout_ms_;
}

uint32_t PcFanController::newest_input_age_ms_() const {
  uint32_t newest = 0;

  if (this->last_cpu_update_ms_ > newest) {
    newest = this->last_cpu_update_ms_;
  }

  if (this->last_gpu_update_ms_ > newest) {
    newest = this->last_gpu_update_ms_;
  }

  if (this->last_other_update_ms_ > newest) {
    newest = this->last_other_update_ms_;
  }

  if (newest == 0) {
    return UINT32_MAX;
  }

  return (uint32_t) (millis() - newest);
}

float PcFanController::curve_pwm_(Channel &channel, float temperature) const {
  if (channel.curve_count == 0) {
    return channel.default_pwm;
  }

  float effective_temperature = temperature;

  if (!std::isnan(channel.last_temp) && temperature < channel.last_temp) {
    effective_temperature += channel.hysteresis;
  }

  channel.last_temp = temperature;

  if (effective_temperature <= channel.curve[0].temp) {
    return channel.curve[0].pwm;
  }

  for (uint8_t i = 1; i < channel.curve_count; i++) {
    const auto &previous = channel.curve[i - 1];
    const auto &current = channel.curve[i];

    if (effective_temperature <= current.temp) {
      const float span = current.temp - previous.temp;

      if (span <= 0.01f) {
        return current.pwm;
      }

      const float ratio = (effective_temperature - previous.temp) / span;
      return previous.pwm + ratio * (current.pwm - previous.pwm);
    }
  }

  return channel.curve[channel.curve_count - 1].pwm;
}

void PcFanController::normalize_curve_(Channel &channel) {
  std::sort(channel.curve.begin(), channel.curve.begin() + channel.curve_count,
            [](const CurvePoint &a, const CurvePoint &b) { return a.temp < b.temp; });

  for (uint8_t i = 0; i < channel.curve_count; i++) {
    channel.curve[i].temp = clamp_(channel.curve[i].temp, 0.0f, 120.0f);
    channel.curve[i].pwm = clamp_(channel.curve[i].pwm, 0.0f, 100.0f);

    if (i > 0 && channel.curve[i].temp <= channel.curve[i - 1].temp) {
      channel.curve[i].temp = channel.curve[i - 1].temp + 1.0f;
    }
  }
}

void PcFanController::set_default_curve_(Channel &channel) {
  channel.curve_count = 4;
  channel.curve[0] = {35.0f, 25.0f};
  channel.curve[1] = {50.0f, 35.0f};
  channel.curve[2] = {65.0f, 60.0f};
  channel.curve[3] = {80.0f, 100.0f};
}

bool PcFanController::apply_curve_json_(Channel &channel, JsonArray curve) {
  uint8_t count = 0;
  auto is_number = [](auto value) {
    return value.template is<float>() || value.template is<int>() || value.template is<long>() ||
           value.template is<unsigned long>();
  };

  for (JsonObject point : curve) {
    if (count >= MAX_CURVE_POINTS) {
      break;
    }

    if (!is_number(point["temp"]) || !is_number(point["pwm"])) {
      continue;
    }

    channel.curve[count].temp = point["temp"].as<float>();
    channel.curve[count].pwm = point["pwm"].as<float>();
    count++;
  }

  if (count < 2) {
    return false;
  }

  channel.curve_count = count;
  this->normalize_curve_(channel);
  return true;
}

bool PcFanController::parse_curve_json_(Channel &channel, const std::string &curve_json) {
  JsonDocument doc = json::parse_json(curve_json);
  if (doc.is<JsonArray>()) {
    return this->apply_curve_json_(channel, doc.as<JsonArray>());
  }

  if (doc.is<JsonObject>() && doc.as<JsonObject>()["curve"].is<JsonArray>()) {
    return this->apply_curve_json_(channel, doc.as<JsonObject>()["curve"].as<JsonArray>());
  }

  return false;
}

PcFanController::Channel *PcFanController::channel_by_id_(uint8_t id) {
  if (id == 0 || id > this->channel_count_) {
    return nullptr;
  }

  return &this->channels_[id - 1];
}

const PcFanController::Channel *PcFanController::channel_by_id_(uint8_t id) const {
  if (id == 0 || id > this->channel_count_) {
    return nullptr;
  }

  return &this->channels_[id - 1];
}

void PcFanController::set_temperature_(InputSource source, float value) {
  value = clamp_(value, -40.0f, 150.0f);

  const uint32_t now = millis();

  if (source == InputSource::CPU) {
    this->input_cpu_ = value;
    this->last_cpu_update_ms_ = now;
  } else if (source == InputSource::GPU) {
    this->input_gpu_ = value;
    this->last_gpu_update_ms_ = now;
  } else if (source == InputSource::OTHER) {
    this->input_other_ = value;
    this->last_other_update_ms_ = now;
  }
}

void PcFanController::set_temperature(const char *source, float value) {
  const InputSource input_source = source_from_string_(source);
  if (input_source == InputSource::MAX) {
    return;
  }

  this->set_temperature_(input_source, value);
  this->regulate_();
}

void PcFanController::set_channel_temperature_sensor(uint8_t channel_id, sensor::Sensor *sensor) {
  Channel *channel = this->channel_by_id_(channel_id);
  if (channel != nullptr) {
    channel->temperature_sensor = sensor;
  }
}

void PcFanController::set_channel_pwm_sensor(uint8_t channel_id, sensor::Sensor *sensor) {
  Channel *channel = this->channel_by_id_(channel_id);
  if (channel != nullptr) {
    channel->pwm_sensor = sensor;
  }
}

void PcFanController::set_channel_status_text_sensor(uint8_t channel_id, text_sensor::TextSensor *sensor) {
  Channel *channel = this->channel_by_id_(channel_id);
  if (channel != nullptr) {
    channel->status_text_sensor = sensor;
  }
}

void PcFanController::save_config_() {
  StoredConfig stored = {};
  stored.magic = PREFERENCE_MAGIC;
  stored.version = PREFERENCE_VERSION;
  stored.channel_count = this->channel_count_;
  stored.data_timeout_ms = this->data_timeout_ms_;
  stored.update_interval_ms = this->update_interval_ms_;

  for (uint8_t i = 0; i < this->channel_count_; i++) {
    const auto &channel = this->channels_[i];
    auto &stored_channel = stored.channels[i];

    copy_string_(stored_channel.name, sizeof(stored_channel.name), channel.name);
    stored_channel.source = static_cast<uint8_t>(channel.source);
    stored_channel.mode = static_cast<uint8_t>(channel.mode);
    stored_channel.inverted = channel.inverted;
    stored_channel.min_pwm = channel.min_pwm;
    stored_channel.max_pwm = channel.max_pwm;
    stored_channel.default_pwm = channel.default_pwm;
    stored_channel.manual_pwm = channel.manual_pwm;
    stored_channel.hysteresis = channel.hysteresis;
    stored_channel.curve_count = channel.curve_count;

    for (uint8_t point_index = 0; point_index < channel.curve_count; point_index++) {
      stored_channel.curve[point_index].temp = channel.curve[point_index].temp;
      stored_channel.curve[point_index].pwm = channel.curve[point_index].pwm;
    }
  }

  this->pref_.save(&stored);

  if (global_preferences != nullptr) {
    global_preferences->sync();
  }
}

void PcFanController::load_config_() {
  StoredConfig stored = {};
  if (!this->pref_.load(&stored)) {
    return;
  }

  if (stored.magic != PREFERENCE_MAGIC || stored.version != PREFERENCE_VERSION) {
    return;
  }

  if (stored.channel_count != this->channel_count_) {
    return;
  }

  this->data_timeout_ms_ = stored.data_timeout_ms;
  this->update_interval_ms_ = stored.update_interval_ms;

  for (uint8_t i = 0; i < this->channel_count_; i++) {
    auto &channel = this->channels_[i];
    const auto &stored_channel = stored.channels[i];

    copy_string_(channel.name, sizeof(channel.name), stored_channel.name);
    channel.source = static_cast<InputSource>(stored_channel.source);
    channel.mode = static_cast<ChannelMode>(stored_channel.mode);
    channel.inverted = stored_channel.inverted;
    channel.min_pwm = clamp_(stored_channel.min_pwm, 0.0f, 100.0f);
    channel.max_pwm = clamp_(stored_channel.max_pwm, channel.min_pwm, 100.0f);
    channel.default_pwm = clamp_(stored_channel.default_pwm, 0.0f, 100.0f);
    channel.manual_pwm = clamp_(stored_channel.manual_pwm, 0.0f, 100.0f);
    channel.hysteresis = clamp_(stored_channel.hysteresis, 0.0f, 20.0f);
    channel.curve_count = std::min<uint8_t>(stored_channel.curve_count, MAX_CURVE_POINTS);

    if (channel.curve_count < 2) {
      this->set_default_curve_(channel);
      continue;
    }

    for (uint8_t point_index = 0; point_index < channel.curve_count; point_index++) {
      channel.curve[point_index].temp = stored_channel.curve[point_index].temp;
      channel.curve[point_index].pwm = stored_channel.curve[point_index].pwm;
    }

    this->normalize_curve_(channel);
  }
}

float PcFanController::clamp_(float value, float min_value, float max_value) {
  if (std::isnan(value)) {
    return min_value;
  }

  if (value < min_value) {
    return min_value;
  }

  if (value > max_value) {
    return max_value;
  }

  return value;
}

bool PcFanController::parse_float_(const std::string &value, float *out) {
  if (out == nullptr) {
    return false;
  }

  char *end = nullptr;
  const float parsed = std::strtof(value.c_str(), &end);

  if (end == value.c_str()) {
    return false;
  }

  *out = parsed;
  return true;
}

bool PcFanController::parse_uint8_(const std::string &value, uint8_t *out) {
  if (out == nullptr) {
    return false;
  }

  char *end = nullptr;
  const long parsed = std::strtol(value.c_str(), &end, 10);

  if (end == value.c_str() || parsed < 0 || parsed > 255) {
    return false;
  }

  *out = static_cast<uint8_t>(parsed);
  return true;
}

PcFanController::InputSource PcFanController::source_from_string_(const char *source) {
  if (source == nullptr) {
    return InputSource::MAX;
  }

  if (strcmp(source, "cpu") == 0) {
    return InputSource::CPU;
  }

  if (strcmp(source, "gpu") == 0) {
    return InputSource::GPU;
  }

  if (strcmp(source, "other") == 0) {
    return InputSource::OTHER;
  }

  return InputSource::MAX;
}

PcFanController::ChannelMode PcFanController::mode_from_string_(const char *mode) {
  if (mode == nullptr) {
    return ChannelMode::AUTO;
  }

  if (strcmp(mode, "manual") == 0) {
    return ChannelMode::MANUAL;
  }

  if (strcmp(mode, "off") == 0) {
    return ChannelMode::OFF;
  }

  return ChannelMode::AUTO;
}

const char *PcFanController::source_to_string_(InputSource source) {
  switch (source) {
    case InputSource::CPU:
      return "cpu";
    case InputSource::GPU:
      return "gpu";
    case InputSource::OTHER:
      return "other";
    case InputSource::MAX:
    default:
      return "max";
  }
}

const char *PcFanController::mode_to_string_(ChannelMode mode) {
  switch (mode) {
    case ChannelMode::MANUAL:
      return "manual";
    case ChannelMode::OFF:
      return "off";
    case ChannelMode::AUTO:
    default:
      return "auto";
  }
}

void PcFanController::copy_string_(char *target, size_t target_size, const char *source) {
  if (target == nullptr || target_size == 0) {
    return;
  }

  if (source == nullptr) {
    target[0] = '\0';
    return;
  }

  strncpy(target, source, target_size - 1);
  target[target_size - 1] = '\0';
}

}  // namespace esphome::pc_fan_controller
