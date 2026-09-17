/* Social Battery — web control surface
 *
 * Two jobs:
 *   1. talk to the Arduino MKR WiFi 1010 over Web Serial (Chrome/Edge, https or localhost)
 *   2. run the same motion model in JS, so the preview moves with or without hardware.
 *      When the board is connected its telemetry wins.
 *
 * Every arm carries its own state, so they can be set independently.
 */

// ---------------------------------------------------------------- config
// These mirror the constants in arduino/move-motors/move-motors.ino.
// If you retune the sketch, retune here too or the preview will drift from the object.
const MAX_ARMS = 3;
const CENTER_ANGLE = [60, 96, 93];
const ANGLE_MIN = 10, ANGLE_MAX = 170;

// Labels for the arm rows. The pin list must match SERVO_PIN[] in the sketch.
// Arms are identified by colour, not by index — that's what's visible on the object.
const ARM_PINS  = ["D5", "D3", "D1"];
const ARM_COLOR = ["Blue", "Yellow", "Red"];
const ARM_SHAPE = ["square", "wedge", "octagon"];
const ARM_CLASS = ["blue", "yellow", "red"];

// Red's level comes from a potentiometer wired straight to the board (see POT_PIN in
// the sketch) — it's the dial's alone to set. The board itself rejects T/X commands
// aimed at it, and "All arms" skips it here too, so the page never even offers to.
//
// The page still shows a dial for it: with no board connected it's a real slider
// driving the same simulation the other arms get, using the same banding math as the
// sketch (POT_BOUND / POT_HYSTERESIS mirror move-motors.ino exactly). Once a board is
// connected it goes read-only and just reflects whatever the real dial + telemetry say.
const POT_ARM = 2;
const POT_BOUND = [256, 512, 768];   // splits 0-1023 into Off | Low | Medium | High
const POT_HYSTERESIS = 25;
const POT_BAND_NAME = ["Off", "Low", "Medium", "High"];
const POT_BAND_MID  = [128, 384, 640, 896];   // representative dial value per band, for display only
let potBand = 0;    // starts at Off, matching every other arm's "stopped" start state
let potValue = 0;   // 0-1023, the dial's raw position while simulating

function potBandFor(reading, current) {
  let band = current;
  while (band < 3 && reading > POT_BOUND[band] + POT_HYSTERESIS) band++;
  while (band > 0 && reading < POT_BOUND[band - 1] - POT_HYSTERESIS) band--;
  return band;
}

// A real rotary pot sweeps 270° with a dead zone at the bottom, not a full circle —
// 0 sits at -135° (about 7 o'clock) and 1023 at +135° (about 5 o'clock), straight up
// being the midpoint of the Low/Medium boundary.
const POT_SWEEP_DEG = 270;
function angleForPotValue(v) { return -135 + (v / 1023) * POT_SWEEP_DEG; }
function potValueForAngle(a) { return clamp(Math.round(((a + 135) / POT_SWEEP_DEG) * 1023), 0, 1023); }

function applyPotBand(band) {
  if (band === potBand) return;
  potBand = band;
  if (band === 0) stopArm(POT_ARM);
  else startArm(POT_ARM, ["LOW", "MED", "HIGH"][band - 1]);
}

// Derives red's current band from its actual state (telemetry when connected, the sim
// model otherwise) rather than from potBand, which only tracks the slider's own drags.
function currentPotBand() {
  const a = arms[POT_ARM];
  if (!a.running) return 0;
  return { LOW: 1, MED: 2, HIGH: 3 }[a.name] ?? 0;
}

// The three states step both amplitude and speed up together: low is slow and
// small, medium a comfortable middle pace, high only slightly faster than medium
// but swinging much wider. Every state swings symmetrically about centre.
const STATES = {
  LOW:  { label: "Low",    amp: 10.0, rate: 0.35, leds: 2 },
  MED:  { label: "Medium", amp: 20.0, rate: 0.90, leds: 4 },
  HIGH: { label: "High",   amp: 35.0, rate: 1.10, leds: 6 },
};
const STATE_BLEND_SEC = 1.4;
const SLEW_DEG_PER_SEC = 140;

// One shared phase clock per state (not per arm) — any arms sharing a state stay in
// lock step, however/whenever each one joined it. Runs continuously in the
// background so a newly-joining arm lines up with whatever's already swinging.
const statePhase = { LOW: 0, MED: 0, HIGH: 0 };

// How a servo angle maps to rotation on screen. The SVG is drawn in the resting pose,
// so rotation is measured from CENTER_ANGLE: each arm swings about its own socket,
// in its own direction, opening the fan wider as the angle rises.
const VIS_SCALE = 1.0;
const VIS_DIR   = [1.0, 1.0, 1.0];
const PIVOT     = [[203, 462], [210, 468], [217, 472]];  // back, middle, front

// ---------------------------------------------------------------- state
const arms = [0, 1, 2].map((i) => ({
  name: "MED",                 // this arm's state
  running: false,              // nothing moves until a state is tapped
  angle: CENTER_ANGLE[i],
  amp: 0,                      // smoothed toward the state's amplitude
}));

const state = { connected: false, armCount: 1 };

const $ = (s) => document.querySelector(s);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const liveArms = () => (state.connected ? state.armCount : MAX_ARMS);

// ---------------------------------------------------------------- serial
let port = null, writer = null, readAbort = false;
let rxBuffer = "";
let helloResolve = null;

const logEl = $("#log");
function log(msg, cls = "") {
  const atBottom = logEl.scrollHeight - logEl.scrollTop - logEl.clientHeight < 24;
  const line = document.createElement("span");
  line.className = cls;
  line.textContent = msg + "\n";
  logEl.appendChild(line);
  while (logEl.childNodes.length > 300) logEl.removeChild(logEl.firstChild);
  if (atBottom) logEl.scrollTop = logEl.scrollHeight;
}

/* Ask the board to identify itself, and keep asking for a few seconds.
 *
 * Opening the port is not proof that an Arduino is on the other end — the OS will
 * happily hand over a Bluetooth modem or a board with no sketch on it. Only a reply
 * to H counts as connected. The retry loop also covers boards that reset when the
 * port opens and need a moment before they can answer. */
async function handshake(timeoutMs = 6000) {
  const deadline = performance.now() + timeoutMs;
  let attempt = 0;
  while (performance.now() < deadline) {
    attempt++;
    log(`checking for a board… (${attempt})`);
    const reply = await new Promise((resolve) => {
      helloResolve = resolve;
      send("H");
      setTimeout(() => resolve(null), 700);
    });
    helloResolve = null;
    if (reply) return reply;
  }
  return null;
}

async function connect() {
  if (!("serial" in navigator)) {
    log("Web Serial is not available. Use Chrome or Edge, over http://localhost or https.", "err");
    return;
  }
  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
    writer = port.writable.getWriter();
    readLoop();
    setConnUI("checking");

    const hello = await handshake();
    if (!hello) {
      log("no answer from that port — nothing identified itself as the board.", "err");
      log("check: sketch uploaded? Serial Monitor closed? right port picked?", "err");
      await disconnect({ quiet: true });
      return;
    }

    const n = Number((hello.match(/ARMS=(\d+)/) || [])[1]);
    if (n) state.armCount = n;
    state.connected = true;
    setConnUI(true);
    log(hello);
    log(`board confirmed, ${state.armCount} servo${state.armCount === 1 ? "" : "s"} — watch the built-in LED blink`);
    paintState();
  } catch (err) {
    log("connect failed: " + err.message, "err");
    await disconnect({ quiet: true });
  }
}

async function disconnect({ quiet = false } = {}) {
  try {
    if (state.connected && writer) send("X");   // leave the object at rest, not mid-swing
    readAbort = true;
    if (writer) { await writer.ready.catch(() => {}); writer.releaseLock(); writer = null; }
    if (port) { await port.close(); port = null; }
  } catch (err) {
    log("disconnect: " + err.message, "err");
  }
  state.connected = false;
  setConnUI(false);
  paintState();
  if (!quiet) log("disconnected");
}

async function readLoop() {
  readAbort = false;
  const decoder = new TextDecoderStream();
  const closed = port.readable.pipeTo(decoder.writable).catch(() => {});
  const reader = decoder.readable.getReader();
  try {
    while (!readAbort) {
      const { value, done } = await reader.read();
      if (done) break;
      rxBuffer += value;
      let i;
      while ((i = rxBuffer.indexOf("\n")) >= 0) {
        handleLine(rxBuffer.slice(0, i).trim());
        rxBuffer = rxBuffer.slice(i + 1);
      }
    }
  } catch (err) {
    log("read error: " + err.message, "err");
  } finally {
    reader.releaseLock();
    await closed;
  }
}

function handleLine(line) {
  if (!line) return;

  if (line.startsWith("OK:HELLO") || line.startsWith("OK:READY")) {
    if (helloResolve) helloResolve(line);
    return;
  }

  if (line.startsWith("S:")) {
    // telemetry: one "<state>,<run>,<angle>" group per arm, ';' separated.
    // The board is the source of truth for anything it reports.
    line.slice(2).split(";").forEach((group, i) => {
      if (i >= MAX_ARMS) return;
      const p = group.split(",");
      if (p.length < 3) return;
      if (STATES[p[0]]) arms[i].name = p[0];
      arms[i].running = p[1] === "1";
      const v = parseFloat(p[2]);
      if (!Number.isNaN(v)) arms[i].angle = v;
    });
    paintState();
    return; // telemetry is 10 Hz — too chatty for the log
  }

  log(line, line.startsWith("ERR") ? "err" : "");
}

function send(cmd) {
  // Say plainly when a command goes nowhere. Logging it as though it were sent is how
  // you end up tapping a button and wondering why the servo did not move.
  if (!writer) { log(`\u2298 ${cmd}   not sent — no board connected`, "warn"); return; }
  log("> " + cmd, "tx");
  writer.write(new TextEncoder().encode(cmd + "\n")).catch((e) => log("write: " + e.message, "err"));
}

function setConnUI(mode) {
  const dot = $("#statusDot"), btn = $("#connectBtn");
  dot.classList.toggle("live", mode === true);
  dot.classList.toggle("checking", mode === "checking");
  $("#connLabel").textContent =
    mode === true ? "board connected" : mode === "checking" ? "checking board…" : "simulating";
  btn.textContent = mode === true ? "Disconnect" : mode === "checking" ? "Checking…" : "Connect board";
  btn.disabled = mode === "checking";
  btn.classList.toggle("primary", mode !== true);
  $("#simNote").hidden = mode === true;
  potControl.classList.toggle("disabled", mode === true);   // real dial takes over once a board answers
  if (mode !== true) {
    // Dropping back to simulation: pick up the dial right where reality left it,
    // rather than snapping to wherever it was last dragged before connecting.
    potBand = currentPotBand();
    potValue = POT_BAND_MID[potBand];
  }
}

// ---------------------------------------------------------------- commands
function startArm(i, name) {
  const a = arms[i];
  // Starting from stopped: take the state's amplitude immediately rather than fading
  // it in, so a tap moves the motor now. No phase to reset any more — reading straight
  // off statePhase[name] is what puts this arm in lock step with any other arm already
  // in that state. Switching states on a running arm leaves amplitude alone, so the
  // swing width eases across without jumping; the phase read just switches clocks.
  if (!a.running) a.amp = STATES[name].amp;
  a.name = name;
  a.running = true;
  paintState();
  send(`T:${i}:${name}`);
}

function startAll(name) {
  for (let i = 0; i < liveArms(); i++) {
    if (i === POT_ARM) continue;   // the dial's alone to set red — "all" means all the rest
    const a = arms[i];
    if (!a.running) a.amp = STATES[name].amp;
    a.name = name;
    a.running = true;
  }
  paintState();
  send(`T:${name}`);
}

function stopArm(i) { arms[i].running = false; paintState(); send(`X:${i}`); }
function stopAll() {
  arms.forEach((a, i) => { if (i !== POT_ARM) a.running = false; });
  paintState();
  send("X");
}

// ---------------------------------------------------------------- motion model
function stepModel(dt) {
  // Every state's clock advances every tick, whether or not an arm is currently using
  // it — that's what lets an arm joining a state mid-cycle land in step immediately
  // instead of resetting the state's cycle to zero for everyone already in it.
  for (const key in statePhase) statePhase[key] += STATES[key].rate * dt;

  const k = clamp(dt / STATE_BLEND_SEC, 0, 1);
  const maxStep = SLEW_DEG_PER_SEC * dt;

  arms.forEach((a, i) => {
    const s = STATES[a.name];
    a.amp += ((a.running ? s.amp : 0) - a.amp) * k;   // stopping fades the swing out
    // Phase isn't eased per arm at all — it's read straight from that state's shared
    // clock, which is exactly what keeps every arm in a state moving as one.
    const want = clamp(
      a.running ? CENTER_ANGLE[i] + Math.sin(statePhase[a.name]) * a.amp
                : CENTER_ANGLE[i],
      ANGLE_MIN, ANGLE_MAX
    );
    a.angle += clamp(want - a.angle, -maxStep, maxStep);
  });
}

// ---------------------------------------------------------------- arm rows
const rowsEl = $("#armRows");
arms.forEach((_, i) => {
  const row = document.createElement("div");
  row.className = "arm-row";
  row.dataset.arm = i;

  // Red has a physical dial wired to the board instead of buttons. With no board
  // connected, this dial IS the real thing — drag it and red follows, same banding math
  // as the sketch. Once a board is connected it goes read-only and just shows what the
  // real dial + telemetry report, since nothing here can actually reach the real ADC.
  // The ring is drawn in four arcs so the Off/Low/Medium/High angle ranges are visible
  // at a glance, not just implied by a linear slider.
  const controls = i === POT_ARM
    ? `<div class="pot-control">
         <svg class="pot-dial" viewBox="0 0 140 140" width="72" height="72">
           <path class="pot-arc off"  d="M36.06,103.94 A48,48 0 0 1 25.65,51.63"/>
           <path class="pot-arc low"  d="M25.65,51.63 A48,48 0 0 1 70,22"/>
           <path class="pot-arc med"  d="M70,22 A48,48 0 0 1 114.35,51.63"/>
           <path class="pot-arc high" d="M114.35,51.63 A48,48 0 0 1 103.94,103.94"/>
           <text class="pot-tick" x="11.15" y="81.71">Off</text>
           <text class="pot-tick" x="36.66" y="20.11">Low</text>
           <text class="pot-tick" x="103.34" y="20.11">Med</text>
           <text class="pot-tick" x="128.85" y="81.71">Hi</text>
           <g class="pot-needle-group">
             <line class="pot-needle" x1="70" y1="70" x2="70" y2="32"/>
           </g>
           <circle class="pot-hub" cx="70" cy="70" r="4"/>
         </svg>
         <small class="pot-band">Off</small>
       </div>`
    : `<div class="seg" data-arm="${i}">
         <button data-state="LOW">Low</button>
         <button data-state="MED">Medium</button>
         <button data-state="HIGH">High</button>
       </div>
       <button class="btn ghost tiny" data-stop="${i}">Stop</button>`;

  row.innerHTML = `
    <span class="arm-id">
      <i class="swatch ${ARM_CLASS[i]}"></i>${ARM_COLOR[i]}
      <small>${ARM_SHAPE[i]} · ${ARM_PINS[i]}</small>
    </span>
    ${controls}
    <span class="arm-live"><b class="deg">—</b><small class="params"></small></span>`;
  rowsEl.appendChild(row);
});

const potControl = rowsEl.querySelector(".pot-control");
const potDial = rowsEl.querySelector(".pot-dial");
const potNeedleGroup = rowsEl.querySelector(".pot-needle-group");
const potBandLabel = rowsEl.querySelector(".pot-band");

// Dragging anywhere on the dial sets the angle from the pointer's position relative to
// its centre — clicking straight down on it jumps there immediately, same as a real
// knob would under a fingertip, and dragging follows continuously from there.
function potValueFromPointer(evt) {
  const rect = potDial.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  const angle = clamp(
    Math.atan2(evt.clientX - cx, -(evt.clientY - cy)) * 180 / Math.PI,
    -135, 135
  );
  return potValueForAngle(angle);
}

potDial.addEventListener("pointerdown", (evt) => {
  if (state.connected) return;   // read-only once a board is actually driving red
  // Capture is what lets the drag keep tracking past the dial's own edge — but it can
  // throw for a pointer the browser doesn't consider active, and a value this central
  // to the interaction shouldn't hinge on that call succeeding.
  try { potDial.setPointerCapture(evt.pointerId); } catch { /* drag still works without it */ }
  potValue = potValueFromPointer(evt);
  applyPotBand(potBandFor(potValue, potBand));
});
potDial.addEventListener("pointermove", (evt) => {
  if (state.connected || evt.buttons !== 1) return;
  potValue = potValueFromPointer(evt);
  applyPotBand(potBandFor(potValue, potBand));
});

// ---------------------------------------------------------------- render
const armEls = [$("#arm0"), $("#arm1"), $("#arm2")];
const rowEls = [...rowsEl.querySelectorAll(".arm-row")];

// six indicator dots in the base, filling outward from the centre. They follow the
// busiest arm currently running: 2 lit for low, 4 for medium, 6 for high.
const ledsG = $("#leds");
const leds = [];
for (let i = 0; i < 6; i++) {
  const c = document.createElementNS("http://www.w3.org/2000/svg", "circle");
  c.setAttribute("cx", 183 + i * 11);
  c.setAttribute("cy", 486);
  c.setAttribute("r", 3.4);
  ledsG.appendChild(c);
  leds.push(c);
}

function render() {
  arms.forEach((a, i) => {
    const rot = (a.angle - CENTER_ANGLE[i]) * VIS_SCALE * VIS_DIR[i];
    const [px, py] = PIVOT[i];
    armEls[i].setAttribute("transform", `rotate(${(-rot).toFixed(2)} ${px} ${py})`);
    armEls[i].classList.toggle("idle", i >= liveArms());

    const row = rowEls[i];
    // Shown relative to this arm's own resting angle, so Stop always reads 0° —
    // the raw absolute servo command (what CENTER_ANGLE calibrates) stays internal.
    const rel = Math.round(a.angle - CENTER_ANGLE[i]) || 0;   // "|| 0" avoids a stray "-0°"
    row.querySelector(".deg").textContent = rel + "°";
    row.querySelector(".params").textContent = a.running
      ? ` ${a.amp.toFixed(1)}° · ${STATES[a.name].rate.toFixed(2)} rad/s`
      : "";
  });

  const lit = Math.max(0, ...arms.filter((a) => a.running).map((a) => STATES[a.name].leds));
  const first = (leds.length - lit) / 2;
  leds.forEach((c, i) => {
    const on = i >= first && i < first + lit;
    c.setAttribute("fill", on ? "#e03e3e" : "#4a4038");
    c.setAttribute("opacity", on ? 1 : 0.55);
  });

  // The label always reflects reality. The needle follows potValue while simulating
  // (the dial IS what's driving red then), or snaps to the real band's angle once a
  // board is connected — never the reverse.
  const band = currentPotBand();
  potBandLabel.textContent = POT_BAND_NAME[band];
  const needleAngle = state.connected ? angleForPotValue(POT_BAND_MID[band]) : angleForPotValue(potValue);
  potNeedleGroup.setAttribute("transform", `rotate(${needleAngle.toFixed(1)} 70 70)`);
}

function paintState() {
  const live = liveArms();

  rowEls.forEach((row, i) => {
    const a = arms[i];
    row.classList.toggle("off", i >= live);
    if (i === POT_ARM) return;   // no buttons on this row — the dial has no "disabled"
    row.querySelectorAll(".seg button").forEach((b) => {
      b.classList.toggle("on", a.running && b.dataset.state === a.name);
      b.disabled = i >= live;
    });
    row.querySelector("[data-stop]").disabled = !a.running || i >= live;
  });

  // "All arms" only ever touches the non-dial arms, so its own highlighting should only
  // ever look at those — red agreeing or not is beside the point, it's not part of "all".
  const settable = arms.slice(0, live).filter((_, i) => i !== POT_ARM);
  const allSame = settable.length > 0 && settable.every((a) => a.running && a.name === settable[0].name);
  document.querySelectorAll('.arm-row.all .seg button').forEach((b) => {
    b.classList.toggle("on", allSame && b.dataset.state === settable[0]?.name);
  });
  const anyRunning = settable.some((a) => a.running);
  document.querySelector('.arm-row.all [data-stop]').disabled = !anyRunning;

  const n = arms.slice(0, live).filter((a) => a.running).length;
  const label = $("#motorLabel");
  label.textContent = n === 0 ? "all stopped" : `${n} of ${live} running`;
  label.classList.toggle("on", n > 0);
}

// ---------------------------------------------------------------- loop
let lastFrame = performance.now();
function frame(now) {
  let dt = (now - lastFrame) / 1000;
  lastFrame = now;
  dt = Math.min(dt, 0.05);           // a backgrounded tab must not fast-forward the model

  if (state.connected) {
    // telemetry drives the angles; keep easing the amplitude readout so it stays honest
    // (rate is just looked up from STATES now, nothing to ease)
    const k = clamp(dt / STATE_BLEND_SEC, 0, 1);
    arms.forEach((a) => {
      const s = STATES[a.name];
      a.amp += ((a.running ? s.amp : 0) - a.amp) * k;
    });
  } else {
    stepModel(dt);
  }

  render();
  requestAnimationFrame(frame);
}

// ---------------------------------------------------------------- wiring
$("#connectBtn").addEventListener("click", () => (state.connected ? disconnect() : connect()));

document.querySelector(".controls").addEventListener("click", (e) => {
  const seg = e.target.closest(".seg button");
  if (seg) {
    const which = seg.parentElement.dataset.arm;
    if (which === "all") startAll(seg.dataset.state);
    else startArm(Number(which), seg.dataset.state);
    return;
  }
  const stop = e.target.closest("[data-stop]");
  if (stop) {
    if (stop.dataset.stop === "all") stopAll();
    else stopArm(Number(stop.dataset.stop));
  }
});

$("#clearLog").addEventListener("click", () => (logEl.textContent = ""));

// ---------------------------------------------------------------- start
paintState();
if (!("serial" in navigator)) {
  log("Web Serial unavailable in this browser — preview runs in simulation.", "err");
}
requestAnimationFrame(frame);
