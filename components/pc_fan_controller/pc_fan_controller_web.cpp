#include "pc_fan_controller.h"

namespace esphome::pc_fan_controller {

static const char FAN_CONTROL_HTML[] = R"HTML(
<section class="wrap">
  <header class="topbar">
    <div>
      <h1>PC Fan Controller</h1>
      <p>ESP32 nadstavba pro PWM ventilátory s regulací podle CPU, GPU a Other teplot.</p>
    </div>
    <div class="status" id="connectionStatus">Načítám...</div>
  </header>

  <section class="cards">
    <article><span>CPU</span><strong id="cpuTemp">--</strong></article>
    <article><span>GPU</span><strong id="gpuTemp">--</strong></article>
    <article><span>Other</span><strong id="otherTemp">--</strong></article>
    <article><span>Data age</span><strong id="dataAge">--</strong></article>
  </section>

  <section class="panel">
    <h2>Poslat teploty</h2>
    <div class="grid3">
      <label>CPU °C <input id="manualCpu" type="number" min="-40" max="150" step="0.1" value="50"></label>
      <label>GPU °C <input id="manualGpu" type="number" min="-40" max="150" step="0.1" value="45"></label>
      <label>Other °C <input id="manualOther" type="number" min="-40" max="150" step="0.1" value="40"></label>
    </div>
    <button type="button" id="sendTemps" class="primary">Odeslat</button>
  </section>

  <nav id="tabs" class="tabs"></nav>
  <main id="channelEditor"></main>
</section>

<style>
  :root {
    color-scheme: dark light;
    --bg: #111827;
    --panel: #1f2937;
    --panel2: #263244;
    --text: #f9fafb;
    --muted: #9ca3af;
    --border: #374151;
    --accent: #38bdf8;
    --danger: #fb7185;
    --ok: #4ade80;
  }

  @media (prefers-color-scheme: light) {
    :root {
      --bg: #f3f4f6;
      --panel: #ffffff;
      --panel2: #f9fafb;
      --text: #111827;
      --muted: #6b7280;
      --border: #d1d5db;
      --accent: #0284c7;
      --danger: #e11d48;
      --ok: #16a34a;
    }
  }

  body { margin: 0; background: var(--bg); color: var(--text); font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
  .wrap { max-width: 1100px; margin: 0 auto; padding: 20px; }
  .topbar { display: flex; justify-content: space-between; gap: 16px; align-items: center; margin-bottom: 18px; }
  h1, h2 { margin: 0; }
  h1 { font-size: 28px; }
  h2 { font-size: 20px; margin-bottom: 12px; }
  p { margin: 6px 0 0; color: var(--muted); }
  .status { border: 1px solid var(--border); border-radius: 999px; padding: 8px 12px; background: var(--panel); white-space: nowrap; }
  .cards { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; margin-bottom: 16px; }
  .cards article, .panel { background: var(--panel); border: 1px solid var(--border); border-radius: 16px; padding: 14px; }
  .cards span { color: var(--muted); display: block; font-size: 14px; margin-bottom: 5px; }
  .cards strong { font-size: 24px; }
  .panel { margin-bottom: 16px; }
  .grid3 { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; }
  .grid2 { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
  label { display: block; color: var(--muted); font-size: 14px; }
  input, select, textarea { margin-top: 5px; width: 100%; box-sizing: border-box; border: 1px solid var(--border); border-radius: 10px; padding: 9px 10px; background: var(--panel2); color: var(--text); font: inherit; }
  textarea { min-height: 92px; }
  button { border: 1px solid var(--border); color: var(--text); background: var(--panel); border-radius: 12px; padding: 9px 12px; cursor: pointer; }
  button.primary { background: var(--accent); border-color: var(--accent); color: #00131f; }
  .tabs { display: flex; gap: 8px; overflow-x: auto; padding-bottom: 8px; margin-bottom: 12px; }
  .tabs button.active { background: var(--accent); border-color: var(--accent); color: #00131f; }
  .curve-table { width: 100%; border-collapse: collapse; margin-top: 12px; }
  .curve-table th, .curve-table td { border-bottom: 1px solid var(--border); padding: 8px; text-align: left; }
  .curve-table th { color: var(--muted); font-weight: 500; }
  .actions { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
  .status-line { margin-top: 10px; color: var(--muted); }
  .ok { color: var(--ok); }
  .danger { color: var(--danger); }
  @media (max-width: 720px) {
    .topbar { align-items: flex-start; flex-direction: column; }
    .cards, .grid3, .grid2 { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  }
  @media (max-width: 460px) {
    .cards, .grid3, .grid2 { grid-template-columns: 1fr; }
  }
</style>

<script>
  const apiBase = "/fan-api";
  let config = null;
  let status = null;
  let activeIndex = 0;

  const tabs = document.getElementById("tabs");
  const editor = document.getElementById("channelEditor");

  function clamp(value, min, max) {
    if (!Number.isFinite(value)) return min;
    return Math.min(max, Math.max(min, value));
  }

  function escapeHtml(value) {
    return String(value)
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;");
  }

  function tempText(value) {
    if (value === null || value === undefined || Number.isNaN(value)) return "--";
    return `${Number(value).toFixed(1)} °C`;
  }

  function pwmText(value) {
    if (value === null || value === undefined || Number.isNaN(value)) return "--";
    return `${Math.round(value)} %`;
  }

  function msText(value) {
    if (value === null || value === undefined || Number.isNaN(value)) return "--";
    if (value < 1000) return `${Math.round(value)} ms`;
    return `${Math.round(value / 1000)} s`;
  }

  async function apiGet(path) {
    const response = await fetch(`${apiBase}${path}`, { cache: "no-store" });
    if (!response.ok) throw new Error(`GET ${path} failed`);
    return await response.json();
  }

  async function apiPost(path, payload) {
    const body = new URLSearchParams();
    body.set("json", JSON.stringify(payload));
    const response = await fetch(`${apiBase}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded" },
      body
    });
    if (!response.ok) throw new Error(`POST ${path} failed`);
    return response;
  }

  function normalizeCurve(channel) {
    channel.curve.sort((a, b) => a.temp - b.temp);
    for (let i = 1; i < channel.curve.length; i++) {
      if (channel.curve[i].temp <= channel.curve[i - 1].temp) {
        channel.curve[i].temp = channel.curve[i - 1].temp + 1;
      }
      channel.curve[i].temp = clamp(channel.curve[i].temp, 20, 100);
      channel.curve[i].pwm = clamp(channel.curve[i].pwm, 0, 100);
    }
    if (channel.curve.length > 0) {
      channel.curve[0].temp = clamp(channel.curve[0].temp, 20, 100);
      channel.curve[0].pwm = clamp(channel.curve[0].pwm, 0, 100);
    }
  }

  async function loadConfig() {
    config = await apiGet("/config");
    renderTabs();
    renderEditor();
  }

  async function loadStatus() {
    status = await apiGet("/status");
    const connection = document.getElementById("connectionStatus");
    connection.textContent = status.online ? "ONLINE" : "FAILSAFE";
    connection.className = status.online ? "status ok" : "status danger";
    document.getElementById("cpuTemp").textContent = tempText(status.inputs.cpu);
    document.getElementById("gpuTemp").textContent = tempText(status.inputs.gpu);
    document.getElementById("otherTemp").textContent = tempText(status.inputs.other);
    document.getElementById("dataAge").textContent = msText(status.inputs.newest_age_ms);
    renderEditor();
  }

  function renderTabs() {
    tabs.innerHTML = "";
    config.channels.forEach((channel, index) => {
      const button = document.createElement("button");
      button.type = "button";
      button.textContent = `${index + 1}. ${channel.name}`;
      button.className = index === activeIndex ? "active" : "";
      button.addEventListener("click", () => {
        activeIndex = index;
        renderTabs();
        renderEditor();
      });
      tabs.appendChild(button);
    });
  }

  function renderEditor() {
    if (!config || !config.channels.length) return;

    const channel = config.channels[activeIndex];
    const channelStatus = status?.channels?.find((item) => item.id === channel.id);

    editor.innerHTML = `
      <section class="panel">
        <h2>${escapeHtml(channel.name)}</h2>

        <div class="grid3">
          <label>Název <input id="channelName" type="text" maxlength="31" value="${escapeHtml(channel.name)}"></label>
          <label>Režim
            <select id="channelMode">
              <option value="auto">Auto</option>
              <option value="manual">Manual</option>
              <option value="off">Off</option>
            </select>
          </label>
          <label>Zdroj teploty
            <select id="channelSource">
              <option value="cpu">CPU</option>
              <option value="gpu">GPU</option>
              <option value="other">Other</option>
              <option value="max">Max</option>
            </select>
          </label>
          <label>Min PWM % <input id="minPwm" type="number" min="0" max="100" step="1" value="${channel.min_pwm}"></label>
          <label>Max PWM % <input id="maxPwm" type="number" min="0" max="100" step="1" value="${channel.max_pwm}"></label>
          <label>Výchozí PWM % <input id="defaultPwm" type="number" min="0" max="100" step="1" value="${channel.default_pwm}"></label>
          <label>Failsafe PWM % <input id="failsafePwm" type="number" min="0" max="100" step="1" value="${channel.failsafe_pwm}"></label>
          <label>Manual PWM % <input id="manualPwm" type="number" min="0" max="100" step="1" value="${channel.manual_pwm}"></label>
          <label>Hystereze °C <input id="hysteresis" type="number" min="0" max="20" step="0.5" value="${channel.hysteresis}"></label>
          <label>Invertovaný výstup
            <select id="inverted">
              <option value="true">Ano</option>
              <option value="false">Ne</option>
            </select>
          </label>
        </div>

        <p class="status-line">
          Stav: <strong>${escapeHtml(channelStatus?.status ?? "--")}</strong>,
          aktuální PWM: <strong>${pwmText(channelStatus?.applied_pwm)}</strong>,
          zdrojová teplota: <strong>${tempText(channelStatus?.source_temp)}</strong>
        </p>

        <table class="curve-table">
          <thead>
            <tr><th>Bod</th><th>Teplota °C</th><th>PWM %</th></tr>
          </thead>
          <tbody id="curveRows"></tbody>
        </table>

        <div class="actions">
          <button type="button" id="addPoint">Přidat bod</button>
          <button type="button" id="removePoint">Odebrat poslední bod</button>
          <button type="button" id="saveConfig" class="primary">Uložit do ESP</button>
          <button type="button" id="reloadConfig">Načíst z ESP</button>
        </div>
      </section>
    `;

    document.getElementById("channelMode").value = channel.mode;
    document.getElementById("channelSource").value = channel.source;
    document.getElementById("inverted").value = String(channel.inverted);

    ["channelName", "channelMode", "channelSource", "minPwm", "maxPwm", "defaultPwm", "failsafePwm", "manualPwm", "hysteresis", "inverted"]
      .forEach((id) => document.getElementById(id).addEventListener("input", updateChannelFromForm));

    document.getElementById("addPoint").addEventListener("click", () => {
      if (channel.curve.length >= 8) return;
      const last = channel.curve[channel.curve.length - 1] ?? { temp: 35, pwm: 25 };
      channel.curve.push({ temp: Math.min(100, last.temp + 5), pwm: Math.min(100, last.pwm + 5) });
      normalizeCurve(channel);
      renderEditor();
    });

    document.getElementById("removePoint").addEventListener("click", () => {
      if (channel.curve.length <= 2) return;
      channel.curve.pop();
      renderEditor();
    });

    document.getElementById("saveConfig").addEventListener("click", async () => {
      updateChannelFromForm();
      await apiPost("/config", config);
      await loadConfig();
      await loadStatus();
    });

    document.getElementById("reloadConfig").addEventListener("click", loadConfig);

    renderCurveTable(channel);
  }

  function renderCurveTable(channel) {
    const tbody = document.getElementById("curveRows");
    if (!tbody) return;

    tbody.innerHTML = "";
    channel.curve.forEach((point, index) => {
      const row = document.createElement("tr");
      row.innerHTML = `
        <td>${index + 1}</td>
        <td><input type="number" min="20" max="100" step="1" value="${point.temp}" data-kind="temp" data-index="${index}"></td>
        <td><input type="number" min="0" max="100" step="1" value="${point.pwm}" data-kind="pwm" data-index="${index}"></td>
      `;
      tbody.appendChild(row);
    });

    tbody.querySelectorAll("input").forEach((input) => {
      input.addEventListener("input", (event) => {
        const target = event.target;
        const index = Number(target.dataset.index);
        const kind = target.dataset.kind;
        const value = Number(target.value);

        if (!Number.isFinite(value) || !channel.curve[index]) return;
        channel.curve[index][kind] = kind === "temp" ? clamp(value, 20, 100) : clamp(value, 0, 100);
        normalizeCurve(channel);
        renderEditor();
      });
    });
  }

  function updateChannelFromForm() {
    const channel = config.channels[activeIndex];
    channel.name = document.getElementById("channelName").value.trim() || channel.name;
    channel.mode = document.getElementById("channelMode").value;
    channel.source = document.getElementById("channelSource").value;
    channel.min_pwm = clamp(Number(document.getElementById("minPwm").value), 0, 100);
    channel.max_pwm = clamp(Number(document.getElementById("maxPwm").value), 0, 100);
    channel.default_pwm = clamp(Number(document.getElementById("defaultPwm").value), 0, 100);
    channel.failsafe_pwm = clamp(Number(document.getElementById("failsafePwm").value), 0, 100);
    channel.manual_pwm = clamp(Number(document.getElementById("manualPwm").value), 0, 100);
    channel.hysteresis = clamp(Number(document.getElementById("hysteresis").value), 0, 20);
    channel.inverted = document.getElementById("inverted").value === "true";

    if (channel.min_pwm > channel.max_pwm) {
      channel.max_pwm = channel.min_pwm;
      document.getElementById("maxPwm").value = channel.max_pwm;
    }
  }

  document.getElementById("sendTemps").addEventListener("click", async () => {
    await apiPost("/temperature", {
      cpu: Number(document.getElementById("manualCpu").value),
      gpu: Number(document.getElementById("manualGpu").value),
      other: Number(document.getElementById("manualOther").value)
    });
    await loadStatus();
  });

  loadConfig().catch((error) => {
    document.getElementById("connectionStatus").textContent = error.message;
    document.getElementById("connectionStatus").className = "status danger";
  });

  loadStatus().catch(() => {});

  setInterval(() => {
    loadStatus().catch(() => {
      document.getElementById("connectionStatus").textContent = "OFFLINE";
      document.getElementById("connectionStatus").className = "status danger";
    });
  }, 1000);
</script>
)HTML";

bool PcFanController::canHandle(AsyncWebServerRequest *request) const {
  char url_buffer[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buffer);

  if (url == this->ui_path_) {
    return request->method() == HTTP_GET;
  }

  if (url == this->api_path_ + "/config") {
    return request->method() == HTTP_GET || request->method() == HTTP_POST;
  }

  if (url == this->api_path_ + "/status") {
    return request->method() == HTTP_GET;
  }

  if (url == this->api_path_ + "/temperature") {
    return request->method() == HTTP_POST;
  }

  if (url == this->api_path_ + "/channel") {
    return request->method() == HTTP_POST;
  }

  return false;
}

void PcFanController::handleRequest(AsyncWebServerRequest *request) {
  char url_buffer[AsyncWebServerRequest::URL_BUF_SIZE];
  const auto url = request->url_to(url_buffer);

  if (url == this->ui_path_) {
    this->handle_ui_(request);
    return;
  }

  if (url == this->api_path_ + "/config") {
    this->handle_api_config_(request);
    return;
  }

  if (url == this->api_path_ + "/status") {
    this->handle_api_status_(request);
    return;
  }

  if (url == this->api_path_ + "/temperature") {
    this->handle_api_temperature_(request);
    return;
  }

  if (url == this->api_path_ + "/channel") {
    this->handle_api_channel_(request);
    return;
  }

  request->send(404);
}

std::string PcFanController::build_config_json_() const {
  return json::build_json([&](JsonObject root) {
    root["data_timeout_ms"] = this->data_timeout_ms_;
    root["update_interval_ms"] = this->update_interval_ms_;
    root["pwm_frequency"] = this->pwm_frequency_;

    JsonArray channels = root["channels"].to<JsonArray>();

    for (uint8_t i = 0; i < this->channel_count_; i++) {
      const auto &channel = this->channels_[i];
      JsonObject channel_json = channels.add<JsonObject>();

      channel_json["id"] = channel.id;
      channel_json["name"] = channel.name;
      channel_json["pin"] = channel.pin->get_pin();
      channel_json["ledc_channel"] = channel.ledc_channel;
      channel_json["inverted"] = channel.inverted;
      channel_json["source"] = source_to_string_(channel.source);
      channel_json["mode"] = mode_to_string_(channel.mode);
      channel_json["min_pwm"] = channel.min_pwm;
      channel_json["max_pwm"] = channel.max_pwm;
      channel_json["default_pwm"] = channel.default_pwm;
      channel_json["failsafe_pwm"] = channel.failsafe_pwm;
      channel_json["manual_pwm"] = channel.manual_pwm;
      channel_json["hysteresis"] = channel.hysteresis;

      JsonArray curve = channel_json["curve"].to<JsonArray>();
      for (uint8_t point_index = 0; point_index < channel.curve_count; point_index++) {
        JsonObject point = curve.add<JsonObject>();
        point["temp"] = channel.curve[point_index].temp;
        point["pwm"] = channel.curve[point_index].pwm;
      }
    }
  });
}

std::string PcFanController::build_status_json_() const {
  return json::build_json([&](JsonObject root) {
    const uint32_t newest_age = this->newest_input_age_ms_();

    JsonObject inputs = root["inputs"].to<JsonObject>();
    inputs["cpu"] = std::isnan(this->input_cpu_) ? nullptr : JsonVariant(this->input_cpu_);
    inputs["gpu"] = std::isnan(this->input_gpu_) ? nullptr : JsonVariant(this->input_gpu_);
    inputs["other"] = std::isnan(this->input_other_) ? nullptr : JsonVariant(this->input_other_);
    inputs["newest_age_ms"] = newest_age == UINT32_MAX ? nullptr : JsonVariant(newest_age);

    root["online"] = this->input_fresh_(this->last_cpu_update_ms_) || this->input_fresh_(this->last_gpu_update_ms_) ||
                      this->input_fresh_(this->last_other_update_ms_);

    JsonArray channels = root["channels"].to<JsonArray>();

    for (uint8_t i = 0; i < this->channel_count_; i++) {
      const auto &channel = this->channels_[i];
      JsonObject channel_json = channels.add<JsonObject>();

      channel_json["id"] = channel.id;
      channel_json["name"] = channel.name;
      channel_json["source"] = source_to_string_(channel.source);
      channel_json["mode"] = mode_to_string_(channel.mode);
      channel_json["applied_pwm"] = channel.applied_pwm;
      channel_json["source_temp"] = std::isnan(channel.source_temp) ? nullptr : JsonVariant(channel.source_temp);
      channel_json["failsafe"] = channel.failsafe;
      channel_json["setup_ok"] = channel.setup_ok;
      channel_json["status"] = channel.status;
    }
  });
}

bool PcFanController::apply_config_json_(const std::string &data) {
  bool ok = false;

  json::parse_json(data, [&](JsonObject root) {
    if (root["data_timeout_ms"].is<uint32_t>()) {
      this->data_timeout_ms_ = root["data_timeout_ms"].as<uint32_t>();
    }

    if (root["update_interval_ms"].is<uint32_t>()) {
      this->update_interval_ms_ = root["update_interval_ms"].as<uint32_t>();
    }

    if (root["channels"].is<JsonArray>()) {
      for (JsonObject channel_json : root["channels"].as<JsonArray>()) {
        if (!channel_json["id"].is<uint8_t>()) {
          continue;
        }

        Channel *channel = this->channel_by_id_(channel_json["id"].as<uint8_t>());
        if (channel == nullptr) {
          continue;
        }

        if (channel_json["name"].is<const char *>()) {
          copy_string_(channel->name, sizeof(channel->name), channel_json["name"].as<const char *>());
        }

        if (channel_json["source"].is<const char *>()) {
          channel->source = source_from_string_(channel_json["source"].as<const char *>());
        }

        if (channel_json["mode"].is<const char *>()) {
          channel->mode = mode_from_string_(channel_json["mode"].as<const char *>());
        }

        if (channel_json["inverted"].is<bool>()) {
          channel->inverted = channel_json["inverted"].as<bool>();
        }

        if (channel_json["min_pwm"].is<float>()) {
          channel->min_pwm = clamp_(channel_json["min_pwm"].as<float>(), 0.0f, 100.0f);
        }

        if (channel_json["max_pwm"].is<float>()) {
          channel->max_pwm = clamp_(channel_json["max_pwm"].as<float>(), channel->min_pwm, 100.0f);
        }

        if (channel_json["default_pwm"].is<float>()) {
          channel->default_pwm = clamp_(channel_json["default_pwm"].as<float>(), 0.0f, 100.0f);
        }

        if (channel_json["failsafe_pwm"].is<float>()) {
          channel->failsafe_pwm = clamp_(channel_json["failsafe_pwm"].as<float>(), 0.0f, 100.0f);
        }

        if (channel_json["manual_pwm"].is<float>()) {
          channel->manual_pwm = clamp_(channel_json["manual_pwm"].as<float>(), 0.0f, 100.0f);
        }

        if (channel_json["hysteresis"].is<float>()) {
          channel->hysteresis = clamp_(channel_json["hysteresis"].as<float>(), 0.0f, 20.0f);
        }

        if (channel_json["curve"].is<JsonArray>()) {
          this->apply_curve_json_(*channel, channel_json["curve"].as<JsonArray>());
        }
      }
    }

    ok = true;
  });

  if (ok) {
    this->save_config_();
    this->regulate_();
  }

  return ok;
}

bool PcFanController::apply_temperature_json_(const std::string &data) {
  bool ok = false;

  json::parse_json(data, [&](JsonObject root) {
    if (root["cpu"].is<float>()) {
      this->set_temperature_(InputSource::CPU, root["cpu"].as<float>());
      ok = true;
    }

    if (root["gpu"].is<float>()) {
      this->set_temperature_(InputSource::GPU, root["gpu"].as<float>());
      ok = true;
    }

    if (root["other"].is<float>()) {
      this->set_temperature_(InputSource::OTHER, root["other"].as<float>());
      ok = true;
    }
  });

  if (ok) {
    this->regulate_();
  }

  return ok;
}

void PcFanController::handle_ui_(AsyncWebServerRequest *request) {
  request->send(200, "text/html; charset=utf-8", FAN_CONTROL_HTML);
}

void PcFanController::handle_api_config_(AsyncWebServerRequest *request) {
  if (request->method() == HTTP_GET) {
    const std::string data = this->build_config_json_();
    request->send(200, "application/json", data.c_str());
    return;
  }

  if (!request->hasArg("json")) {
    request->send(409, "application/json", "{\"error\":\"missing json form field\"}");
    return;
  }

  const std::string data = request->arg("json");

  if (!this->apply_config_json_(data)) {
    request->send(409, "application/json", "{\"error\":\"invalid config json\"}");
    return;
  }

  request->send(200, "application/json", "{\"ok\":true}");
}

void PcFanController::handle_api_status_(AsyncWebServerRequest *request) {
  const std::string data = this->build_status_json_();
  request->send(200, "application/json", data.c_str());
}

void PcFanController::handle_api_temperature_(AsyncWebServerRequest *request) {
  bool ok = false;

  if (request->hasArg("json")) {
    ok = this->apply_temperature_json_(request->arg("json"));
  } else {
    float value = NAN;

    if (request->hasArg("cpu") && parse_float_(request->arg("cpu"), &value)) {
      this->set_temperature_(InputSource::CPU, value);
      ok = true;
    }

    if (request->hasArg("gpu") && parse_float_(request->arg("gpu"), &value)) {
      this->set_temperature_(InputSource::GPU, value);
      ok = true;
    }

    if (request->hasArg("other") && parse_float_(request->arg("other"), &value)) {
      this->set_temperature_(InputSource::OTHER, value);
      ok = true;
    }

    if (ok) {
      this->regulate_();
    }
  }

  if (!ok) {
    request->send(409, "application/json", "{\"error\":\"missing or invalid temperature\"}");
    return;
  }

  request->send(200, "application/json", "{\"ok\":true}");
}

void PcFanController::handle_api_channel_(AsyncWebServerRequest *request) {
  uint8_t id = 0;

  if (!request->hasArg("id") || !parse_uint8_(request->arg("id"), &id)) {
    request->send(409, "application/json", "{\"error\":\"missing id\"}");
    return;
  }

  Channel *channel = this->channel_by_id_(id);
  if (channel == nullptr) {
    request->send(404, "application/json", "{\"error\":\"unknown channel\"}");
    return;
  }

  if (request->hasArg("mode")) {
    channel->mode = mode_from_string_(request->arg("mode").c_str());
  }

  float value = NAN;
  if (request->hasArg("manual_pwm") && parse_float_(request->arg("manual_pwm"), &value)) {
    channel->manual_pwm = clamp_(value, 0.0f, 100.0f);
  }

  this->save_config_();
  this->regulate_();

  request->send(200, "application/json", "{\"ok\":true}");
}

}  // namespace esphome::pc_fan_controller
