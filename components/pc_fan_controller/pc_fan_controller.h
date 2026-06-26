#pragma once

#include <array>
#include <cstdint>
#include <string>

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"

#include "esphome/components/json/json_util.h"
#include "esphome/components/web_server_base/web_server_base.h"

#ifdef USE_ESP32
#include "driver/ledc.h"
#endif

#ifndef USE_PC_FAN_CONTROLLER_MAX_CHANNELS
#define USE_PC_FAN_CONTROLLER_MAX_CHANNELS 8
#endif

namespace esphome::pc_fan_controller {

class PcFanController : public Component, public AsyncWebHandler {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_update_interval_ms(uint32_t update_interval_ms) { this->update_interval_ms_ = update_interval_ms; }
  void set_data_timeout_ms(uint32_t data_timeout_ms) { this->data_timeout_ms_ = data_timeout_ms; }
  void set_pwm_frequency(uint32_t pwm_frequency) { this->pwm_frequency_ = pwm_frequency; }
  void set_ui_path(const std::string &ui_path) { this->ui_path_ = ui_path; }
  void set_api_path(const std::string &api_path) { this->api_path_ = api_path; }

  void add_channel(InternalGPIOPin *pin, const char *name, uint8_t ledc_channel, bool inverted, const char *source,
                   float min_pwm, float max_pwm, float default_pwm, float failsafe_pwm, float manual_pwm, float hysteresis,
                   const char *curve_json);

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  bool isRequestHandlerTrivial() const override { return false; }

 protected:
  static constexpr uint8_t MAX_CURVE_POINTS = 8;
  static constexpr uint32_t PREFERENCE_MAGIC = 0x50434643;
  static constexpr uint32_t PREFERENCE_VERSION = 2;
  static constexpr uint32_t PWM_DUTY_MAX = 1023;

  enum class InputSource : uint8_t {
    CPU = 0,
    GPU = 1,
    OTHER = 2,
    MAX = 3,
  };

  enum class ChannelMode : uint8_t {
    AUTO = 0,
    MANUAL = 1,
    OFF = 2,
  };

  enum class TemperatureState : uint8_t {
    FRESH = 0,
    MISSING = 1,
    STALE = 2,
  };

  struct CurvePoint {
    float temp{0.0f};
    float pwm{0.0f};
  };

  struct Channel {
    InternalGPIOPin *pin{nullptr};
    char name[32]{};
    uint8_t id{0};
    uint8_t ledc_channel{0};
    bool inverted{true};
    InputSource source{InputSource::MAX};
    ChannelMode mode{ChannelMode::AUTO};
    float min_pwm{25.0f};
    float max_pwm{100.0f};
    float default_pwm{50.0f};
    float failsafe_pwm{100.0f};
    float manual_pwm{50.0f};
    float hysteresis{3.0f};
    float applied_pwm{0.0f};
    float source_temp{NAN};
    float last_temp{NAN};
    bool setup_ok{false};
    bool failsafe{true};
    char status[80]{};
    std::array<CurvePoint, MAX_CURVE_POINTS> curve{};
    uint8_t curve_count{0};
  };

  struct StoredCurvePoint {
    float temp;
    float pwm;
  };

  struct StoredChannel {
    char name[32];
    uint8_t source;
    uint8_t mode;
    bool inverted;
    float min_pwm;
    float max_pwm;
    float default_pwm;
    float failsafe_pwm;
    float manual_pwm;
    float hysteresis;
    uint8_t curve_count;
    StoredCurvePoint curve[MAX_CURVE_POINTS];
  };

  struct StoredConfig {
    uint32_t magic;
    uint32_t version;
    uint8_t channel_count;
    uint32_t data_timeout_ms;
    uint32_t update_interval_ms;
    StoredChannel channels[USE_PC_FAN_CONTROLLER_MAX_CHANNELS];
  };

  std::array<Channel, USE_PC_FAN_CONTROLLER_MAX_CHANNELS> channels_{};
  uint8_t channel_count_{0};

  uint32_t update_interval_ms_{500};
  uint32_t data_timeout_ms_{10000};
  uint32_t pwm_frequency_{25000};
  uint32_t last_regulation_ms_{0};

  std::string ui_path_{"/fan-control"};
  std::string api_path_{"/fan-api"};

  float input_cpu_{NAN};
  float input_gpu_{NAN};
  float input_other_{NAN};
  uint32_t last_cpu_update_ms_{0};
  uint32_t last_gpu_update_ms_{0};
  uint32_t last_other_update_ms_{0};

  ESPPreferenceObject pref_{};

  void setup_ledc_();
  void regulate_();
  void regulate_channel_(Channel &channel);
  void set_channel_pwm_(Channel &channel, float pwm);
  uint32_t duty_for_pwm_(const Channel &channel, float pwm) const;

  TemperatureState source_temperature_(InputSource source, float *temperature) const;
  bool input_fresh_(uint32_t stamp) const;
  uint32_t newest_input_age_ms_() const;

  float curve_pwm_(Channel &channel, float temperature) const;
  void normalize_curve_(Channel &channel);
  void set_default_curve_(Channel &channel);
  bool apply_curve_json_(Channel &channel, JsonArray curve);
  bool parse_curve_json_(Channel &channel, const std::string &curve_json);

  Channel *channel_by_id_(uint8_t id);
  const Channel *channel_by_id_(uint8_t id) const;

  void set_temperature_(InputSource source, float value);

  bool apply_config_json_(const std::string &data);
  bool apply_temperature_json_(const std::string &data);
  std::string build_config_json_() const;
  std::string build_status_json_() const;

  void save_config_();
  void load_config_();

  void handle_ui_(AsyncWebServerRequest *request);
  void handle_api_config_(AsyncWebServerRequest *request);
  void handle_api_status_(AsyncWebServerRequest *request);
  void handle_api_temperature_(AsyncWebServerRequest *request);
  void handle_api_channel_(AsyncWebServerRequest *request);

  static float clamp_(float value, float min_value, float max_value);
  static bool parse_float_(const std::string &value, float *out);
  static bool parse_uint8_(const std::string &value, uint8_t *out);
  static InputSource source_from_string_(const char *source);
  static ChannelMode mode_from_string_(const char *mode);
  static const char *source_to_string_(InputSource source);
  static const char *mode_to_string_(ChannelMode mode);
  static void copy_string_(char *target, size_t target_size, const char *source);
};

}  // namespace esphome::pc_fan_controller
