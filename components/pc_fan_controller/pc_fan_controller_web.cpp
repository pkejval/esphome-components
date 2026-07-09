#include "pc_fan_controller.h"

namespace esphome::pc_fan_controller {

static std::string html_escape(std::string value) {
  size_t pos = 0;
  while ((pos = value.find('&', pos)) != std::string::npos) {
    value.replace(pos, 1, "&amp;");
    pos += 5;
  }
  pos = 0;
  while ((pos = value.find('<', pos)) != std::string::npos) {
    value.replace(pos, 1, "&lt;");
    pos += 4;
  }
  pos = 0;
  while ((pos = value.find('>', pos)) != std::string::npos) {
    value.replace(pos, 1, "&gt;");
    pos += 4;
  }
  pos = 0;
  while ((pos = value.find('"', pos)) != std::string::npos) {
    value.replace(pos, 1, "&quot;");
    pos += 6;
  }
  return value;
}

static const char FAN_CONTROL_HTML[] = R"HTML(
<section class="wrap">
  <header class="topbar">
    <div>
      <h1 id="pageTitle">__TITLE__</h1>
    </div>
    <div class="status" id="connectionStatus">Načítám...</div>
  </header>

  <section class="cards">
    <article><span>CPU</span><strong id="cpuTemp">--</strong></article>
    <article><span>GPU</span><strong id="gpuTemp">--</strong></article>
    <article><span>Other</span><strong id="otherTemp">--</strong></article>
    <article><span>Data age</span><strong id="dataAge">--</strong></article>
  </section>

  <nav id="tabs" class="tabs"></nav>
  <main id="channelEditor"></main>

  <details class="panel send-temps">
    <summary>Poslat teploty</summary>
    <div class="grid3">
      <label>CPU °C <input id="manualCpu" type="number" min="-40" max="150" step="0.1" value="50"></label>
      <label>GPU °C <input id="manualGpu" type="number" min="-40" max="150" step="0.1" value="45"></label>
      <label>Other °C <input id="manualOther" type="number" min="-40" max="150" step="0.1" value="40"></label>
    </div>
    <button type="button" id="sendTemps" class="primary">Odeslat</button>
  </details>
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
  .send-temps summary { cursor: pointer; font-weight: 600; list-style: none; }
  .send-temps summary::-webkit-details-marker { display: none; }
  .send-temps summary::before { content: ">"; display: inline-block; margin-right: 8px; color: var(--muted); transition: transform 0.15s ease; }
  .send-temps[open] summary::before { transform: rotate(90deg); }
  .send-temps > .grid3 { margin-top: 12px; }
  .send-temps > button { margin-top: 12px; }
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
  .curve-graph {
    position: relative;
    width: 100%;
    height: 420px;
    margin-top: 12px;
    border: 1px solid var(--border);
    border-radius: 16px;
    background: linear-gradient(180deg, rgba(56, 189, 248, 0.04), transparent), var(--panel2);
    overflow: hidden;
    touch-action: none;
  }
  .curve-graph svg {
    width: 100%;
    height: 100%;
    display: block;
    shape-rendering: geometricPrecision;
    text-rendering: geometricPrecision;
  }
  .graph-add {
    position: absolute;
    top: 10px;
    right: 10px;
    z-index: 2;
    padding: 6px 10px;
    border-radius: 999px;
    background: rgba(17, 24, 39, 0.78);
    backdrop-filter: blur(8px);
  }
  .graph-add:hover { border-color: var(--accent); }
  .curve-point { cursor: grab; }
  .curve-point:active { cursor: grabbing; }
  .curve-point.is-hovered circle:last-child { fill: #facc15; }
  .curve-point.is-hovered .curve-halo { fill: rgba(250, 204, 21, 0.18); }
  .curve-point.is-endpoint circle:last-child { fill: #60a5fa; }
  .curve-segment { stroke: transparent; stroke-width: 2.2; pointer-events: stroke; cursor: crosshair; vector-effect: non-scaling-stroke; }
  .curve-segment.is-hovered { stroke: rgba(250, 204, 21, 0.9); }
  .curve-label { fill: var(--muted); font-size: 4.5px; }
  .curve-grid-minor { stroke: var(--border); stroke-width: 0.22; opacity: 0.18; vector-effect: non-scaling-stroke; shape-rendering: crispEdges; }
  .curve-grid-major { stroke: var(--border); stroke-width: 0.4; opacity: 0.45; vector-effect: non-scaling-stroke; shape-rendering: crispEdges; }
  .curve-axis { stroke: var(--muted); stroke-width: 0.65; opacity: 0.9; vector-effect: non-scaling-stroke; shape-rendering: crispEdges; }
  .curve-line { stroke: var(--accent); stroke-width: 1.8; fill: none; stroke-linecap: round; stroke-linejoin: round; vector-effect: non-scaling-stroke; }
  .curve-halo { fill: rgba(56, 189, 248, 0.14); }
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
  let curveDrag = null;
  let curveHover = null;
  let applyTimer = null;
  const GRAPH_WIDTH = 560;
  const GRAPH_HEIGHT = 260;
  const GRAPH_MARGIN = { left: 28, right: 18, top: 16, bottom: 34 };

  const tabs = document.getElementById("tabs");
  const editor = document.getElementById("channelEditor");

  document.title = document.getElementById("pageTitle")?.textContent ?? "PC Fan Controller";

  function clamp(value, min, max) {
    if (!Number.isFinite(value)) return min;
    return Math.min(max, Math.max(min, value));
  }

  function roundInt(value) {
    return Math.round(value);
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
    return `${roundInt(value)} °C`;
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

  async function applyDraft() {
    if (!config) return;
    updateChannelFromForm();
    await apiPost("/apply", config);
    await loadStatus();
  }

  function scheduleApply() {
    if (applyTimer !== null) {
      clearTimeout(applyTimer);
    }

    applyTimer = setTimeout(async () => {
      applyTimer = null;
      try {
        await applyDraft();
      } catch (error) {
        console.error(error);
      }
    }, 80);
  }

  function normalizeCurve(channel) {
    channel.curve.sort((a, b) => a.temp - b.temp);
    for (let i = 1; i < channel.curve.length; i++) {
      if (channel.curve[i].temp <= channel.curve[i - 1].temp) {
        channel.curve[i].temp = channel.curve[i - 1].temp + 1;
      }
      channel.curve[i].temp = roundInt(clamp(channel.curve[i].temp, 0, 100));
      channel.curve[i].pwm = roundInt(clamp(channel.curve[i].pwm, 0, 100));
    }
    if (channel.curve.length > 0) {
      channel.curve[0].temp = roundInt(clamp(channel.curve[0].temp, 0, 100));
      channel.curve[0].pwm = roundInt(clamp(channel.curve[0].pwm, 0, 100));
    }
  }

  function curveToSvgPoint(point) {
    const plotWidth = GRAPH_WIDTH - GRAPH_MARGIN.left - GRAPH_MARGIN.right;
    const plotHeight = GRAPH_HEIGHT - GRAPH_MARGIN.top - GRAPH_MARGIN.bottom;
    return {
      x: GRAPH_MARGIN.left + clamp(point.temp, 0, 100) * plotWidth / 100,
      y: GRAPH_MARGIN.top + clamp(100 - point.pwm, 0, 100) * plotHeight / 100
    };
  }

  function svgToCurvePoint(x, y) {
    const plotWidth = GRAPH_WIDTH - GRAPH_MARGIN.left - GRAPH_MARGIN.right;
    const plotHeight = GRAPH_HEIGHT - GRAPH_MARGIN.top - GRAPH_MARGIN.bottom;
    return {
      temp: roundInt(clamp((x - GRAPH_MARGIN.left) * 100 / plotWidth, 0, 100)),
      pwm: roundInt(clamp((GRAPH_HEIGHT - GRAPH_MARGIN.bottom - y) * 100 / plotHeight, 0, 100))
    };
  }

  function getSvgPoint(svg, event) {
    const rect = svg.getBoundingClientRect();
    return {
      x: clamp(((event.clientX - rect.left) / rect.width) * GRAPH_WIDTH, 0, GRAPH_WIDTH),
      y: clamp(((event.clientY - rect.top) / rect.height) * GRAPH_HEIGHT, 0, GRAPH_HEIGHT)
    };
  }

  function dist2(ax, ay, bx, by) {
    const dx = ax - bx;
    const dy = ay - by;
    return dx * dx + dy * dy;
  }

  function projectPointOnSegment(px, py, ax, ay, bx, by) {
    const abx = bx - ax;
    const aby = by - ay;
    const length2 = abx * abx + aby * aby;

    if (length2 <= 0.0001) {
      return { x: ax, y: ay, t: 0, d2: dist2(px, py, ax, ay) };
    }

    const t = clamp(((px - ax) * abx + (py - ay) * aby) / length2, 0, 1);
    const x = ax + t * abx;
    const y = ay + t * aby;
    return { x, y, t, d2: dist2(px, py, x, y) };
  }

  function hitTestCurve(channel, x, y) {
    const pointThreshold2 = 10 * 10;
    const segmentThreshold2 = 1.5 * 1.5;
    let nearestPoint = null;
    let nearestSegment = null;

    channel.curve.forEach((point, index) => {
      const svgPoint = curveToSvgPoint(point);
      const d2 = dist2(x, y, svgPoint.x, svgPoint.y);
      if (nearestPoint === null || d2 < nearestPoint.d2) {
        nearestPoint = { kind: "point", index, d2 };
      }
    });

    for (let i = 0; i < channel.curve.length - 1; i++) {
      const a = curveToSvgPoint(channel.curve[i]);
      const b = curveToSvgPoint(channel.curve[i + 1]);
      const projected = projectPointOnSegment(x, y, a.x, a.y, b.x, b.y);
      const nearEndpoint = projected.t < 0.18 || projected.t > 0.82;
      if (!nearEndpoint && (nearestSegment === null || projected.d2 < nearestSegment.d2)) {
        nearestSegment = { kind: "segment", index: i, ...projected };
      }
    }

    const pointHit = nearestPoint !== null && nearestPoint.d2 <= pointThreshold2 ? nearestPoint : null;
    const segmentHit = nearestSegment !== null && nearestSegment.d2 <= segmentThreshold2 ? nearestSegment : null;

    if (pointHit !== null && (segmentHit === null || pointHit.d2 <= segmentHit.d2)) {
      return pointHit;
    }

    return segmentHit;
  }

  function insertPointOnSegment(channel, segmentIndex, point) {
    channel.curve.splice(segmentIndex + 1, 0, svgToCurvePoint(point.x, point.y));
    normalizeCurve(channel);
    return segmentIndex + 1;
  }

  function addGraphPoint(channel) {
    if (channel.curve.length < 2) return null;

    let segmentIndex = 0;
    let longestSpan = -1;

    for (let i = 0; i < channel.curve.length - 1; i++) {
      const span = channel.curve[i + 1].temp - channel.curve[i].temp;
      if (span > longestSpan) {
        longestSpan = span;
        segmentIndex = i;
      }
    }

    const a = curveToSvgPoint(channel.curve[segmentIndex]);
    const b = curveToSvgPoint(channel.curve[segmentIndex + 1]);
    return insertPointOnSegment(channel, segmentIndex, { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 });
  }

  async function loadConfig() {
    config = await apiGet("/config");
    renderTabs();
    renderEditor();
  }

  async function loadStatus() {
    status = await apiGet("/status");
    updateStatusWidgets();
  }

  function updateStatusWidgets() {
    const connection = document.getElementById("connectionStatus");
    if (connection !== null) {
      connection.textContent = status.online ? "ONLINE" : "OFFLINE";
      connection.className = status.online ? "status ok" : "status danger";
    }

    const cpuTemp = document.getElementById("cpuTemp");
    if (cpuTemp !== null) cpuTemp.textContent = tempText(status.inputs.cpu);

    const gpuTemp = document.getElementById("gpuTemp");
    if (gpuTemp !== null) gpuTemp.textContent = tempText(status.inputs.gpu);

    const otherTemp = document.getElementById("otherTemp");
    if (otherTemp !== null) otherTemp.textContent = tempText(status.inputs.other);

    const dataAge = document.getElementById("dataAge");
    if (dataAge !== null) dataAge.textContent = msText(status.inputs.newest_age_ms);

    const channel = config?.channels?.[activeIndex];
    if (channel !== undefined) {
      const channelStatus = status?.channels?.find((item) => item.id === channel.id);

      const statusText = document.getElementById("channelStatusText");
      if (statusText !== null) statusText.textContent = channelStatus?.status ?? "--";

      const appliedPwm = document.getElementById("channelAppliedPwm");
      if (appliedPwm !== null) appliedPwm.textContent = pwmText(channelStatus?.applied_pwm);

      const sourceTemp = document.getElementById("channelSourceTemp");
      if (sourceTemp !== null) sourceTemp.textContent = tempText(channelStatus?.source_temp);
    }
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
    const manualOnly = channel.mode === "manual";

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
          <label style="${manualOnly ? "display:none;" : ""}">Zdroj teploty
            <select id="channelSource">
              <option value="cpu">CPU</option>
              <option value="gpu">GPU</option>
              <option value="other">Other</option>
              <option value="max">Max</option>
            </select>
          </label>
          <label style="${manualOnly ? "display:none;" : ""}">Min PWM % <input id="minPwm" type="number" min="0" max="100" step="1" value="${channel.min_pwm}"></label>
          <label style="${manualOnly ? "display:none;" : ""}">Max PWM % <input id="maxPwm" type="number" min="0" max="100" step="1" value="${channel.max_pwm}"></label>
          <label style="${manualOnly ? "display:none;" : ""}">Výchozí PWM % <input id="defaultPwm" type="number" min="0" max="100" step="1" value="${channel.default_pwm}"></label>
          <label>Manual PWM % <input id="manualPwm" type="number" min="0" max="100" step="1" value="${channel.manual_pwm}"></label>
          <label style="${manualOnly ? "display:none;" : ""}">Hystereze °C <input id="hysteresis" type="number" min="0" max="20" step="0.5" value="${channel.hysteresis}"></label>
          <label>Invertovaný výstup
            <select id="inverted">
              <option value="true">Ano</option>
              <option value="false">Ne</option>
            </select>
          </label>
        </div>

        <p class="status-line">
          Stav: <strong id="channelStatusText">${escapeHtml(channelStatus?.status ?? "--")}</strong>,
          aktuální PWM: <strong id="channelAppliedPwm">${pwmText(channelStatus?.applied_pwm)}</strong>${manualOnly ? "" : `,
          zdrojová teplota: <strong id="channelSourceTemp">${tempText(channelStatus?.source_temp)}</strong>`}
        </p>

        <div class="curve-graph" id="curveGraph" style="${manualOnly ? "display:none;" : ""}"></div>

        <table class="curve-table" style="${manualOnly ? "display:none;" : ""}">
          <thead>
            <tr><th>Bod</th><th>Teplota °C</th><th>PWM %</th></tr>
          </thead>
          <tbody id="curveRows"></tbody>
        </table>

        <div class="actions">
          <button type="button" id="addPoint" style="${manualOnly ? "display:none;" : ""}">Přidat bod</button>
          <button type="button" id="removePoint" style="${manualOnly ? "display:none;" : ""}">Odebrat poslední bod</button>
          <button type="button" id="saveConfig" class="primary">Uložit do ESP</button>
          <button type="button" id="reloadConfig">Načíst z ESP</button>
        </div>
      </section>
    `;

    document.getElementById("channelMode").value = channel.mode;
    document.getElementById("channelSource").value = channel.source;
    document.getElementById("inverted").value = String(channel.inverted);

    ["channelName", "channelSource", "minPwm", "maxPwm", "defaultPwm", "manualPwm", "hysteresis", "inverted"]
      .forEach((id) => document.getElementById(id).addEventListener("input", scheduleApply));

    document.getElementById("channelMode").addEventListener("change", (event) => {
      updateChannelFromForm(event);
      scheduleApply();
      renderEditor();
    });

    document.getElementById("addPoint").addEventListener("click", () => {
      if (channel.curve.length >= 8) return;
      const last = channel.curve[channel.curve.length - 1] ?? { temp: 35, pwm: 25 };
      channel.curve.push({ temp: Math.min(100, last.temp + 5), pwm: Math.min(100, last.pwm + 5) });
      normalizeCurve(channel);
      scheduleApply();
      renderEditor();
    });

    document.getElementById("removePoint").addEventListener("click", () => {
      if (channel.curve.length <= 2) return;
      channel.curve.pop();
      scheduleApply();
      renderEditor();
    });

    document.getElementById("saveConfig").addEventListener("click", async () => {
      updateChannelFromForm();
      await apiPost("/config", config);
      await loadConfig();
      await loadStatus();
    });

    document.getElementById("reloadConfig").addEventListener("click", loadConfig);

    renderCurveGraph(channel);
    renderCurveTable(channel);
  }

  function renderCurveGraph(channel) {
    const container = document.getElementById("curveGraph");
    if (container === null) return;

    const points = channel.curve.map(curveToSvgPoint);
    const path = points.length > 0
      ? `M ${points.map((point) => `${point.x},${point.y}`).join(" L ")}`
      : "";

    const plotWidth = GRAPH_WIDTH - GRAPH_MARGIN.left - GRAPH_MARGIN.right;
    const plotHeight = GRAPH_HEIGHT - GRAPH_MARGIN.top - GRAPH_MARGIN.bottom;

    const grid = [];
    for (let i = 0; i <= 100; i++) {
      const x = GRAPH_MARGIN.left + (i * plotWidth / 100);
      const y = GRAPH_MARGIN.top + (i * plotHeight / 100);
      const cls = i % 10 === 0 ? "curve-grid-major" : "curve-grid-minor";
      grid.push(`<line class="${cls}" x1="${x}" y1="${GRAPH_MARGIN.top}" x2="${x}" y2="${GRAPH_HEIGHT - GRAPH_MARGIN.bottom}"></line>`);
      grid.push(`<line class="${cls}" x1="${GRAPH_MARGIN.left}" y1="${y}" x2="${GRAPH_WIDTH - GRAPH_MARGIN.right}" y2="${y}"></line>`);
    }

    const axisLabels = [];
    for (let i = 0; i <= 10; i++) {
      const value = i * 10;
      const x = GRAPH_MARGIN.left + (value * plotWidth / 100);
      const y = GRAPH_MARGIN.top + ((100 - value) * plotHeight / 100);
      axisLabels.push(`<text class="curve-label" x="${x}" y="${GRAPH_HEIGHT - 6}" text-anchor="middle">${value}</text>`);
      axisLabels.push(`<text class="curve-label" x="4" y="${y + 2}" text-anchor="start">${value}</text>`);
    }

    const segments = points.slice(0, -1).map((point, index) => {
      const next = points[index + 1];
      const hovered = curveHover?.kind === "segment" && curveHover.index === index ? "is-hovered" : "";
      return `<line class="curve-segment ${hovered}" data-segment="${index}" x1="${point.x}" y1="${point.y}" x2="${next.x}" y2="${next.y}"></line>`;
    }).join("");

    container.innerHTML = `
      <button type="button" id="addGraphPoint" class="graph-add">+ bod</button>
      <svg id="curveGraphSvg" viewBox="0 0 ${GRAPH_WIDTH} ${GRAPH_HEIGHT}" preserveAspectRatio="none" aria-label="PWM curve graph">
        <rect x="0" y="0" width="${GRAPH_WIDTH}" height="${GRAPH_HEIGHT}" fill="transparent"></rect>
        ${grid.join("")}
        <line class="curve-axis" x1="${GRAPH_MARGIN.left}" y1="${GRAPH_HEIGHT - GRAPH_MARGIN.bottom}" x2="${GRAPH_WIDTH - GRAPH_MARGIN.right}" y2="${GRAPH_HEIGHT - GRAPH_MARGIN.bottom}"></line>
        <line class="curve-axis" x1="${GRAPH_MARGIN.left}" y1="${GRAPH_MARGIN.top}" x2="${GRAPH_MARGIN.left}" y2="${GRAPH_HEIGHT - GRAPH_MARGIN.bottom}"></line>
        <path class="curve-line" d="${path}" data-role="curve-path"></path>
        ${segments}
        ${points.map((point, index) => `
          <g class="curve-point ${curveHover?.kind === "point" && curveHover.index === index ? "is-hovered" : ""} ${curveHover?.kind === "segment" && (curveHover.index === index || curveHover.index + 1 === index) ? "is-endpoint" : ""}" data-index="${index}">
            <circle class="curve-halo" cx="${point.x}" cy="${point.y}" r="4.2"></circle>
            <circle fill="var(--accent)" cx="${point.x}" cy="${point.y}" r="1.8"></circle>
          </g>
        `).join("")}
        ${axisLabels.join("")}
      </svg>
    `;

    const addButton = document.getElementById("addGraphPoint");
    if (addButton !== null) {
      addButton.addEventListener("click", () => {
        const insertedIndex = addGraphPoint(channel);
        if (insertedIndex !== null) {
          curveHover = { kind: "point", index: insertedIndex };
          renderCurveGraph(channel);
          renderCurveTable(channel);
        }
      });
    }

    const svg = document.getElementById("curveGraphSvg");
    if (svg === null) return;

    svg.addEventListener("pointermove", (event) => {
      if (curveDrag !== null) return;
      const point = getSvgPoint(svg, event);
      curveHover = hitTestCurve(channel, point.x, point.y);
      renderCurveGraph(channel);
    });

    svg.addEventListener("pointerleave", () => {
      if (curveDrag !== null) return;
      curveHover = null;
      renderCurveGraph(channel);
    });

    svg.addEventListener("pointerdown", (event) => {
      const point = getSvgPoint(svg, event);
      const hit = hitTestCurve(channel, point.x, point.y);
      if (hit === null) return;

      event.preventDefault();

      if (hit.kind === "point") {
        curveDrag = { index: hit.index, pointerId: event.pointerId };
        curveHover = hit;
        svg.setPointerCapture(event.pointerId);
        return;
      }

      if (hit.kind === "segment") {
        const insertedIndex = insertPointOnSegment(channel, hit.index, point);
        curveDrag = { index: insertedIndex, pointerId: event.pointerId };
        curveHover = { kind: "point", index: insertedIndex };
        renderCurveGraph(channel);
        renderCurveTable(channel);
        svg.setPointerCapture(event.pointerId);
      }
    });
  }

  function updateCurveFromPointer(event) {
    if (curveDrag === null) return;

    const channel = config?.channels?.[activeIndex];
    if (channel === undefined || !channel.curve[curveDrag.index]) return;

    const svg = document.getElementById("curveGraphSvg");
    if (svg === null) return;

    const point = getSvgPoint(svg, event);
    const x = point.x;
    const y = point.y;

    channel.curve[curveDrag.index] = svgToCurvePoint(x, y);
    renderCurveGraph(channel);
    renderCurveTable(channel);
  }

  window.addEventListener("pointermove", (event) => {
    if (curveDrag === null) return;
    updateCurveFromPointer(event);
  });

  window.addEventListener("pointerup", () => {
    if (curveDrag !== null) {
      const channel = config?.channels?.[activeIndex];
      if (channel !== undefined) {
        normalizeCurve(channel);
        renderCurveGraph(channel);
        renderCurveTable(channel);
        scheduleApply();
      }
    }
    curveDrag = null;
  });

  window.addEventListener("pointercancel", () => {
    curveDrag = null;
  });

  function renderCurveTable(channel) {
    const tbody = document.getElementById("curveRows");
    if (!tbody) return;

    tbody.innerHTML = "";
    channel.curve.forEach((point, index) => {
      const row = document.createElement("tr");
      row.innerHTML = `
        <td>${index + 1}</td>
        <td><input type="number" min="0" max="100" step="1" value="${point.temp}" data-kind="temp" data-index="${index}"></td>
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
        channel.curve[index][kind] = kind === "temp" ? clamp(value, 0, 100) : clamp(value, 0, 100);
        normalizeCurve(channel);
        renderCurveGraph(channel);
        renderCurveTable(channel);
        scheduleApply();
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

  if (url == this->api_path_ + "/apply") {
    return request->method() == HTTP_POST;
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

  if (url == this->api_path_ + "/apply") {
    this->handle_api_apply_(request);
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
    if (std::isnan(this->input_cpu_)) {
      inputs["cpu"] = nullptr;
    } else {
      inputs["cpu"] = this->input_cpu_;
    }

    if (std::isnan(this->input_gpu_)) {
      inputs["gpu"] = nullptr;
    } else {
      inputs["gpu"] = this->input_gpu_;
    }

    if (std::isnan(this->input_other_)) {
      inputs["other"] = nullptr;
    } else {
      inputs["other"] = this->input_other_;
    }

    if (newest_age == UINT32_MAX) {
      inputs["newest_age_ms"] = nullptr;
    } else {
      inputs["newest_age_ms"] = newest_age;
    }

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
      if (std::isnan(channel.source_temp)) {
        channel_json["source_temp"] = nullptr;
      } else {
        channel_json["source_temp"] = channel.source_temp;
      }
      channel_json["failsafe"] = channel.failsafe;
      channel_json["setup_ok"] = channel.setup_ok;
      channel_json["status"] = channel.status;
    }
  });
}

bool PcFanController::apply_config_json_(const std::string &data, bool persist) {
  bool ok = false;
  auto is_number = [](auto value) {
    return value.template is<float>() || value.template is<int>() || value.template is<long>() ||
           value.template is<unsigned long>();
  };

  JsonDocument doc = json::parse_json(data);
  if (!doc.is<JsonObject>()) {
    return false;
  }

  JsonObject root = doc.as<JsonObject>();

  {
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

        if (is_number(channel_json["min_pwm"])) {
          channel->min_pwm = clamp_(channel_json["min_pwm"].as<float>(), 0.0f, 100.0f);
        }

        if (is_number(channel_json["max_pwm"])) {
          channel->max_pwm = clamp_(channel_json["max_pwm"].as<float>(), channel->min_pwm, 100.0f);
        }

        if (is_number(channel_json["default_pwm"])) {
          channel->default_pwm = clamp_(channel_json["default_pwm"].as<float>(), 0.0f, 100.0f);
        }

        if (is_number(channel_json["manual_pwm"])) {
          channel->manual_pwm = clamp_(channel_json["manual_pwm"].as<float>(), 0.0f, 100.0f);
        }

        if (is_number(channel_json["hysteresis"])) {
          channel->hysteresis = clamp_(channel_json["hysteresis"].as<float>(), 0.0f, 20.0f);
        }

        if (channel_json["curve"].is<JsonArray>()) {
          this->apply_curve_json_(*channel, channel_json["curve"].as<JsonArray>());
        }
      }
    }

    ok = true;
  }

  if (ok && persist) {
    this->save_config_();
  }

  if (ok) {
    this->regulate_();
  }

  return ok;
}

bool PcFanController::apply_temperature_json_(const std::string &data) {
  bool ok = false;
  auto is_number = [](auto value) {
    return value.template is<float>() || value.template is<int>() || value.template is<long>() ||
           value.template is<unsigned long>();
  };

  JsonDocument doc = json::parse_json(data);
  if (!doc.is<JsonObject>()) {
    return false;
  }

  JsonObject root = doc.as<JsonObject>();

  {
    if (is_number(root["cpu"])) {
      this->set_temperature_(InputSource::CPU, root["cpu"].as<float>());
      ok = true;
    }

    if (is_number(root["gpu"])) {
      this->set_temperature_(InputSource::GPU, root["gpu"].as<float>());
      ok = true;
    }

    if (is_number(root["other"])) {
      this->set_temperature_(InputSource::OTHER, root["other"].as<float>());
      ok = true;
    }
  }

  if (ok) {
    this->regulate_();
  }

  return ok;
}

void PcFanController::handle_ui_(AsyncWebServerRequest *request) {
  std::string html = FAN_CONTROL_HTML;
  const std::string title = html_escape(this->ui_title_);

  size_t pos = 0;
  while ((pos = html.find("__TITLE__", pos)) != std::string::npos) {
    html.replace(pos, 9, title);
    pos += title.length();
  }

  request->send(200, "text/html; charset=utf-8", html.c_str());
}

void PcFanController::handle_api_config_(AsyncWebServerRequest *request) {
  if (request->method() == HTTP_GET) {
    const std::string data = this->build_config_json_();
    request->send(200, "application/json", data.c_str());
    return;
  }

  std::string body;
  if (request->hasArg("plain")) {
    body = request->arg("plain");
  } else if (request->hasArg("json")) {
    body = request->arg("json");
  } else {
    request->send(409, "application/json", "{\"error\":\"missing json form field\"}");
    return;
  }

  if (!this->apply_config_json_(body, true)) {
    request->send(409, "application/json", "{\"error\":\"invalid config json\"}");
    return;
  }

  request->send(200, "application/json", "{\"ok\":true}");
}

void PcFanController::handle_api_apply_(AsyncWebServerRequest *request) {
  std::string body;
  if (request->hasArg("plain")) {
    body = request->arg("plain");
  } else if (request->hasArg("json")) {
    body = request->arg("json");
  } else {
    request->send(409, "application/json", "{\"error\":\"missing json form field\"}");
    return;
  }

  if (!this->apply_config_json_(body, false)) {
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

  if (request->hasArg("plain") || request->hasArg("json")) {
    const std::string body = request->hasArg("plain") ? request->arg("plain") : request->arg("json");
    ok = this->apply_temperature_json_(body);
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
