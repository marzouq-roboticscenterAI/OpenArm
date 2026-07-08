"use strict";
/* roverservo dashboard client.
 * Polls /api/state for telemetry and drives both devices via the JSON API.
 * Motion controls are "hold to move": keys/buttons feed a setpoint that the
 * server's deadman auto-stops once we stop refreshing it. */

const PULSES_PER_REV = 10000;
const TELEOP_HZ = 10;            // how often we refresh a held setpoint
const POLL_MS = 150;            // telemetry poll period

// ---- tiny API helpers ---------------------------------------------------- //
async function post(path, body) {
  try {
    const r = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    const j = await r.json().catch(() => ({}));
    if (!r.ok || j.ok === false) flash(j.error || `HTTP ${r.status} on ${path}`);
    return j;
  } catch (e) {
    flash(`network error: ${e.message}`);
    return { ok: false };
  }
}
const $ = (id) => document.getElementById(id);

let bannerTimer = null;
function flash(msg) {
  const b = $("banner");
  b.textContent = msg;
  b.classList.add("show");
  clearTimeout(bannerTimer);
  bannerTimer = setTimeout(() => b.classList.remove("show"), 4000);
}

// ======================================================================== //
// ROVER teleop
// ======================================================================== //
const held = new Set();               // active directions: fwd/back/left/right
let selectedMode = "ackermann";
let lastNonzero = false;

function computeCmd() {
  let linear = 0, angular = 0, steer = 0;
  const lin = parseFloat($("rover-linspeed").value);
  const turn = parseFloat($("rover-turn").value);
  if (selectedMode === "spin") {
    if (held.has("left")) angular += turn;    // CCW positive
    if (held.has("right")) angular -= turn;
  } else if (selectedMode !== "park") {
    if (held.has("fwd")) linear += lin;
    if (held.has("back")) linear -= lin;
    if (held.has("left")) steer += turn;      // left-turn positive
    if (held.has("right")) steer -= turn;
  }
  return { linear, angular, steer };
}

function teleopTick() {
  if (!roverPresent || !roverEnabled) return;
  const cmd = computeCmd();
  const nonzero = cmd.linear || cmd.angular || cmd.steer;
  if (nonzero || lastNonzero) post("/api/rover/drive", cmd);
  lastNonzero = !!nonzero;
}

// keyboard
const KEYMAP = { w: "fwd", s: "back", a: "left", d: "right",
                 arrowup: "fwd", arrowdown: "back", arrowleft: "left", arrowright: "right" };
addEventListener("keydown", (e) => {
  const k = e.key.toLowerCase();
  if (k === " ") { e.preventDefault(); estopAll(); return; }
  if (KEYMAP[k]) { e.preventDefault(); held.add(KEYMAP[k]); syncPad(); }
});
addEventListener("keyup", (e) => {
  const k = e.key.toLowerCase();
  if (KEYMAP[k]) { held.delete(KEYMAP[k]); syncPad(); }
});
addEventListener("blur", () => { held.clear(); syncPad(); });

// on-screen D-pad (pointer = mouse + touch)
function syncPad() {
  document.querySelectorAll("#rover-pad button[data-drive]").forEach((b) => {
    const d = b.dataset.drive;
    b.classList.toggle("held", held.has(d));
  });
}
document.querySelectorAll("#rover-pad button[data-drive]").forEach((b) => {
  const dir = b.dataset.drive;
  const press = (e) => {
    e.preventDefault();
    if (dir === "stop") { held.clear(); post("/api/rover/stop"); }
    else held.add(dir);
    syncPad();
  };
  const release = () => { if (dir !== "stop") { held.delete(dir); syncPad(); } };
  b.addEventListener("pointerdown", press);
  b.addEventListener("pointerup", release);
  b.addEventListener("pointerleave", release);
  b.addEventListener("pointercancel", release);
});

// mode selector
document.querySelectorAll("#rover-modeseg button").forEach((b) => {
  b.addEventListener("click", () => {
    selectedMode = b.dataset.mode;
    document.querySelectorAll("#rover-modeseg button").forEach((x) => x.classList.remove("active"));
    b.classList.add("active");
    held.clear(); syncPad();
    post("/api/rover/mode", { mode: selectedMode });
  });
});

// buttons
$("rover-enable").onclick = () => post("/api/rover/enable");
$("rover-disable").onclick = () => post("/api/rover/disable");
$("rover-clear").onclick = () => post("/api/rover/clear_errors");
let lightsOn = false;
$("rover-light").onclick = () => { lightsOn = !lightsOn; post("/api/rover/light", { on: lightsOn }); };

// sliders
const linSlider = $("rover-linspeed"), turnSlider = $("rover-turn");
linSlider.oninput = () => $("rover-linspeed-val").textContent = (+linSlider.value).toFixed(2) + " m/s";
turnSlider.oninput = () => $("rover-turn-val").textContent = (+turnSlider.value).toFixed(2);

// ======================================================================== //
// SERVO controls
// ======================================================================== //
const stepSlider = $("servo-step"), speedSlider = $("servo-speed");
function servoStep() { return parseInt(stepSlider.value, 10); }
function servoSpeed() { return parseInt(speedSlider.value, 10); }
stepSlider.oninput = () =>
  $("servo-step-val").textContent = (servoStep() / PULSES_PER_REV).toFixed(1) + " rev";
speedSlider.oninput = () =>
  $("servo-speed-val").textContent = Math.round(servoSpeed() * 60 / PULSES_PER_REV) + " rpm";

$("servo-enable").onclick = () => post("/api/servo/enable");
$("servo-disable").onclick = () => post("/api/servo/disable");
$("servo-reset").onclick = () => post("/api/servo/fault_reset");
$("servo-estop").onclick = () => post("/api/servo/estop");
$("servo-raise").onclick = () => post("/api/servo/raise", { pulses: servoStep(), velocity: servoSpeed() });
$("servo-lower").onclick = () => post("/api/servo/lower", { pulses: servoStep(), velocity: servoSpeed() });

// jog hold: refresh velocity while held, stop on release (server deadman backs this up)
function makeJog(btnId, sign) {
  const b = $(btnId);
  let iv = null;
  const start = (e) => {
    e.preventDefault();
    if (iv) return;
    const send = () => post("/api/servo/jog", { velocity: sign * servoSpeed() });
    send();
    iv = setInterval(send, 1000 / TELEOP_HZ);
    b.classList.add("held");
  };
  const stop = () => {
    if (!iv) return;
    clearInterval(iv); iv = null;
    post("/api/servo/stop");
    b.classList.remove("held");
  };
  b.addEventListener("pointerdown", start);
  b.addEventListener("pointerup", stop);
  b.addEventListener("pointerleave", stop);
  b.addEventListener("pointercancel", stop);
}
makeJog("servo-jog-up", -1);    // up = negative velocity (drive polarity is inverted)
makeJog("servo-jog-down", +1);

// ======================================================================== //
// E-STOP ALL
// ======================================================================== //
function estopAll() {
  held.clear(); syncPad();
  post("/api/estop");
  flash("E-STOP sent to both devices");
}
$("estop-all").onclick = estopAll;

// ======================================================================== //
// Telemetry rendering
// ======================================================================== //
let roverPresent = false, roverEnabled = false;

function setDot(id, cls) { $(id).className = "dot " + (cls || ""); }

// Enable/disable every control inside a panel (so an offline device's buttons
// can't be clicked). The E-STOP ALL button is never disabled.
function setPanelEnabled(panelId, enabled) {
  document.querySelectorAll(`#${panelId} button, #${panelId} input`).forEach((el) => {
    el.disabled = !enabled;
  });
}

function renderRover(r) {
  roverPresent = !!(r && r.present);
  setPanelEnabled("rover-panel", roverPresent);
  if (!roverPresent) {
    setDot("rover-dot", "bad");
    $("rover-iface").textContent = "(not connected)";
    return;
  }
  roverEnabled = !!r.enabled;
  $("rover-iface").textContent = r.iface || "";
  const live = r.connected && !r.link_stale;
  setDot("rover-dot", r.estop ? "bad" : (live ? "ok" : "warn"));
  $("rover-batt").textContent = r.voltage != null ? r.voltage.toFixed(1) + " V" : "—";
  $("rover-ctrl").textContent = r.control_mode || "—";
  $("rover-mode").textContent = (r.motion_mode || "—") + (r.switching_mode ? " (switching)" : "");
  const m = r.motion;
  $("rover-speed").textContent = m
    ? `${m.linear.toFixed(2)} m/s · ${m.angular.toFixed(2)} rad/s`
    : "—";
  const fb = $("rover-faults");
  if (r.faults && r.faults.length) fb.innerHTML = r.faults.map((f) => `<span class="f">⚠ ${f}</span>`).join(" · ");
  else fb.innerHTML = live ? '<span class="none">no faults</span>' : '<span class="none">waiting for feedback…</span>';
}

function renderServo(s) {
  const present = !!(s && s.present);
  setPanelEnabled("servo-panel", present);
  if (!present) {
    setDot("servo-dot", "bad");
    $("servo-iface").textContent = "(not connected)";
    $("servo-pos").textContent = $("servo-vel").textContent = "—";
    $("servo-state").textContent = $("servo-status").textContent = "offline";
    $("servo-faults").innerHTML = '<span class="none">servo not connected</span>';
    return;
  }
  $("servo-iface").textContent = (s.iface || "") + (s.node != null ? ` · node ${s.node}` : "");
  const fault = !!s.fault;
  setDot("servo-dot", fault ? "bad" : (s.op_enabled ? "ok" : "warn"));
  $("servo-pos").textContent = s.position_rev != null ? s.position_rev.toFixed(3) + " rev" : "—";
  $("servo-vel").textContent = s.velocity_rpm != null
    ? `${s.velocity_rpm.toFixed(1)} rpm (${s.velocity_pps} pps)` : "—";
  $("servo-state").textContent = s.state || "—";
  $("servo-status").textContent = s.status || "—";
  const fb = $("servo-faults");
  if (fault) fb.innerHTML = '<span class="f">⚠ drive fault — try Fault reset</span>';
  else fb.innerHTML = '<span class="none">' + (s.op_enabled ? "operation enabled" : "not enabled") + "</span>";
}

async function poll() {
  try {
    const r = await fetch("/api/state", { cache: "no-store" });
    const st = await r.json();
    if (st.error) { $("link-sub").textContent = "server error: " + st.error; return; }
    renderRover(st.rover);
    renderServo(st.servo);
    const parts = [];
    if (st.rover && st.rover.present) parts.push("rover " + st.rover.iface);
    if (st.servo && st.servo.present) parts.push("servo " + st.servo.iface);
    $("link-sub").textContent = parts.length ? "connected · " + parts.join(" · ") : "no devices";
  } catch (e) {
    $("link-sub").textContent = "disconnected from server";
  }
}

setInterval(poll, POLL_MS);
setInterval(teleopTick, 1000 / TELEOP_HZ);
poll();
