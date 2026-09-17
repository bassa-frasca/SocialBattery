/*
  move-motors — Social Battery
  ----------------------------
  The whole thing in one sketch. Servo arms on a shared base; each arm carries its own
  state, and the three states step both amplitude and speed up together:

      state    amplitude   speed         reads as
      LOW      10°         slow          present, but holding still
      MEDIUM   20°         medium        clearly swaying, comfortable pace
      HIGH     35°         slightly      fully switched on — wide and lively
                           faster than
                           medium

  Every state swings the arm symmetrically about its resting angle — the same number
  of degrees left and right of centre, whichever state it's in. Only the width and
  pace of that swing change.

  It runs two ways, and you do not have to choose in advance:

    * On its own. Plug the board in and it demonstrates itself, cycling every arm
      through the three states, six seconds each. No computer needed.
    * Driven. The web page (or anything that can open a serial port) takes over the
      moment it sends a command, and the demo cycle stands down.

  The red arm is a special case: a potentiometer on A1 is its permanent, sole controller
  (see the "potentiometer" section below). It never joins the standalone demo and it
  can't be driven by serial/web commands — turning the dial is what it means to set red.

  Board: Arduino MKR WiFi 1010. Serial is the native USB port, 115200 baud.

  ---------------------------------------------------------------- protocol
  One ASCII command per line, '\n' terminated:

    H                  handshake: blink the LED pattern and identify the board
    T:<arm>:<state>    start ONE arm in that state, e.g. T:0:HIGH
    T:<state>          start EVERY OTHER arm in that state, e.g. T:MED (skips POT_ARM)
    X:<arm>            stop one arm
    X                  stop every OTHER arm (skips POT_ARM)
    E:<0|1>            detach / attach the servos by hand
    ?                  send one status line now

  Replies:
    OK:HELLO social-battery ARMS=<n>                     answer to H
    S:<state>,<run>,<angle>;<state>,<run>,<angle>;...    one group per arm, 10 Hz
    OK:<echo>  /  ERR:<reason>

  T:<arm>:<state> and X:<arm> targeting POT_ARM (red, index 2) get ERR:arm is
  dial-controlled instead — see "potentiometer" below.

  ---------------------------------------------------------------- serial monitor
  Typing the protocol by hand is tedious, so single characters work too:

    1 / 2 / 3   all arms to low / medium / high
    0           stop all arms
    a           resume the automatic demo cycle
    c           hold every arm at CENTER_ANGLE, for setting the resting pose

  Digits and lowercase letters were chosen so they cannot collide with the protocol.

  ---------------------------------------------------------------- wiring
  Three servo signals -> D5, D3, D1 (see SERVO_PIN below for which arm is which).
  Servo power from an EXTERNAL 5V supply, its ground tied to the board's ground — do not
  run servos off the MKR's own regulator. One potentiometer -> A1, driving red's level
  directly (see POT_PIN below); its outer legs go to 3.3V and GND, never 5V.
*/

#include <Servo.h>

// ---------------------------------------------------------------- pins
//
// One entry per arm. Add pins here and the whole sketch adapts — per-arm state,
// telemetry and the arm count reported to the web page all size themselves from it.
//
const uint8_t SERVO_PIN[] = { 5, 3, 1 };
//
// Arm order follows this array: index 0 is the blue square on the longest rod at the
// back (D5), 1 the yellow wedge in the middle (D3), 2 the red octagon at the front (D1).
//
// D1 is also the MKR's Serial1 TX pin. This sketch never uses Serial1, so it's free to
// drive a servo — but keep that in mind before adding anything that talks over Serial1.
//
// Avoid pins 8, 9 and 10 on the MKR WiFi 1010 — they are the SPI bus to the onboard
// NINA WiFi module. A servo there works only until something switches the radio on.

const uint8_t MAX_ARMS  = 3;
const uint8_t ARM_COUNT = sizeof(SERVO_PIN) / sizeof(SERVO_PIN[0]);
const uint8_t LED_PIN   = LED_BUILTIN;   // pin 6 on the MKR boards

// The SAMD Servo library drives any digital pin from a hardware timer rather than from
// analogWrite, so the plain digital pins D3, D4 and D5 are all fine here.

// ---------------------------------------------------------------- potentiometer
//
// A physical dial for the red arm's level. Wiring: the pot's two OUTER legs go across
// the divider — one to 3.3V, the other to GND — and the MIDDLE leg (the wiper, i.e. its
// signal/"out" pin) goes to A1, the analog input. Never wire either outer leg to 5V:
// the MKR's ADC reference is 3.3V, and feeding 5V into the divider risks putting more
// than that on an input pin.
//
const uint8_t POT_PIN = A1;
const uint8_t POT_ARM = 2;   // red octagon — the arm this dial drives

// Splits the pot's 0-1023 reading into four even bands: Off | Low | Medium | High. Each
// boundary carries a dead zone POT_HYSTERESIS counts wide, so noise sitting right on a
// line can't flicker the arm back and forth — a reading has to clear the line by that
// margin, in whichever direction, before the band actually changes.
const int POT_BOUND[3]   = { 256, 512, 768 };
const int POT_HYSTERESIS = 25;

// ---------------------------------------------------------------- tuning

// Resting pose — the middle of each arm's swing. State changes how an arm MOVES, not
// where it sits. Three entries, so the others are ready when you add them.
const int CENTER_ANGLE[MAX_ARMS] = { 60, 96, 93 };

// Hard clamp on every movement. Keep inside whatever the linkage can physically reach.
const int ANGLE_MIN = 10;
const int ANGLE_MAX = 170;

enum State { ST_LOW = 0, ST_MED = 1, ST_HIGH = 2, STATE_COUNT = 3 };
const char *STATE_NAME[STATE_COUNT] = { "LOW", "MED", "HIGH" };

// The whole design lives in these two tables. Amplitude is degrees either side of
// centre — the swing is symmetrical, so the arm travels this many degrees each way
// from CENTER_ANGLE; rate is radians per second.
//                                          LOW    MED    HIGH
const float STATE_AMP_DEG[STATE_COUNT]  = { 10.0,  20.0,  35.0 };  // small, medium, large
const float STATE_RATE[STATE_COUNT]     = { 0.35,  0.90,  1.10 };  // slow, medium, slightly faster than medium

// Seconds to cross from one state's amplitude/speed to another's. A state change is a
// mood change, not a switch — it should be readable as it happens.
const float STATE_BLEND_SEC = 1.4;

// Speed limit on every movement, degrees per second. This is what makes the arm read
// as alive rather than as machinery; lower is heavier and more reluctant.
const float SLEW_DEG_PER_SEC = 140.0;

// Seconds per state in the standalone demo cycle.
const unsigned long DWELL_MS = 6000;

// ---------------------------------------------------------------- state

Servo servos[MAX_ARMS];
bool  servosAttached[MAX_ARMS] = { false, false, false };

// Everything below is per arm.
State armState[MAX_ARMS]   = { ST_MED, ST_MED, ST_MED };
bool  armRunning[MAX_ARMS] = { false, false, false };
float armAngle[MAX_ARMS];       // what the servo is actually holding
float armAmp[MAX_ARMS]  = { 0, 0, 0 };   // smoothed toward the state's amplitude

// One shared phase clock per STATE, not per arm — every arm currently in a given
// state reads the same clock, so any two arms sharing a state are always in lock
// step, however and whenever each one joined it. It runs continuously, whether or
// not any arm is using it right now, so a newly-joining arm always lines up with
// whatever's already swinging in that state instead of restarting the cycle.
float statePhase[STATE_COUNT] = { 0, 0, 0 };

bool autoCycle = true;      // stands down as soon as a command arrives
bool holdCenter = false;    // park at CENTER_ANGLE while setting the resting pose
uint8_t cycleState = 0;
unsigned long lastSwitch = 0;

char line[32];
uint8_t lineLen = 0;
unsigned long lastTick = 0, lastReport = 0;

// ---------------------------------------------------------------- led

// On/off durations in ms, alternating, starting with ON. A zero ends the pattern.
// Handshake: three quick blinks, a beat, then one long one — unmistakable across a
// room, and distinct from the bootloader's own flicker at reset.
const uint16_t PATTERN_HELLO[] = { 90, 90, 90, 90, 90, 300, 700, 0 };

uint16_t ledPattern[12];
uint8_t  ledSteps = 0, ledStep = 0;
unsigned long ledStepStart = 0;
bool ledPlaying = false;

void playPattern(const uint16_t *p) {
  const uint8_t cap = sizeof(ledPattern) / sizeof(ledPattern[0]);
  ledSteps = 0;
  while (p[ledSteps] != 0 && ledSteps < cap) { ledPattern[ledSteps] = p[ledSteps]; ledSteps++; }
  ledStep = 0;
  ledStepStart = millis();
  ledPlaying = ledSteps > 0;
}

bool anyRunning() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) if (armRunning[i]) return true;
  return false;
}

void updateLed() {
  if (ledPlaying) {
    if (millis() - ledStepStart >= ledPattern[ledStep]) {
      ledStepStart = millis();
      if (++ledStep >= ledSteps) ledPlaying = false;
    }
    if (ledPlaying) {
      digitalWrite(LED_PIN, (ledStep % 2 == 0) ? HIGH : LOW);   // even steps are ON
      return;
    }
  }
  // no pattern playing: the LED reports whether anything is moving
  digitalWrite(LED_PIN, anyRunning() ? HIGH : LOW);
}

// ---------------------------------------------------------------- helpers

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Case-insensitive compare. Written out rather than using strcasecmp, which the AVR
// and SAMD cores expose from different headers.
bool eq(const char *a, const char *b) {
  while (*a && *b) { if (toupper(*a) != toupper(*b)) return false; a++; b++; }
  return *a == *b;
}

// Returns STATE_COUNT if the name is not one of ours.
uint8_t parseState(const char *n) {
  for (uint8_t i = 0; i < STATE_COUNT; i++) if (eq(n, STATE_NAME[i])) return i;
  if (eq(n, "MEDIUM")) return ST_MED;   // the page spells it out; accept both
  return STATE_COUNT;
}

void attachArm(uint8_t i, bool on) {
  if (on == servosAttached[i]) return;
  if (on) servos[i].attach(SERVO_PIN[i]);
  else    servos[i].detach();
  servosAttached[i] = on;
}

void startArm(uint8_t i, State s) {
  // Starting from stopped: take the state's amplitude immediately rather than fading
  // it in, so a tap moves the motor now instead of in a second (the slew limiter still
  // walks it there rather than snapping). No phase to reset any more — reading straight
  // off statePhase[s] is what puts this arm in lock step with any other arm already in
  // state s. Switching states on an arm already running leaves amplitude alone, so it
  // eases across without jumping; the phase read simply switches to the new state's
  // clock immediately, which is what re-syncs it to that state's other arms.
  if (!armRunning[i]) armAmp[i] = STATE_AMP_DEG[s];
  armState[i] = s;
  armRunning[i] = true;
  holdCenter = false;
  attachArm(i, true);
}

// Both skip POT_ARM once a dial is wired to it — see readPot() below for why.
void startAll(State s) { for (uint8_t i = 0; i < ARM_COUNT; i++) if (i != POT_ARM) startArm(i, s); }

void stopArm(uint8_t i) { armRunning[i] = false; }   // eases home and holds there, powered
void stopAll() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) if (i != POT_ARM) stopArm(i);
  holdCenter = false;
}

// ---------------------------------------------------------------- potentiometer

// -1 means "not read yet" — that forces the very first call to settle on whatever band
// the dial is actually sitting at, rather than assuming it starts at Off.
int8_t potBand = -1;

// The dial is POT_ARM's only source of truth: it doesn't call takeControl(), so turning
// it never stops blue/yellow's standalone demo — startAll()/stopAll() (which is all the
// demo cycle and the "every arm" serial/web commands ever call) both skip POT_ARM on
// their own, and a command aimed at POT_ARM specifically is rejected outright. Once a
// physical dial is wired to an arm, that dial is what it means for that arm to be
// "controlled" — nothing else can step on it.
void readPot() {
  int reading = analogRead(POT_PIN);

  int8_t band = (potBand < 0) ? 0 : potBand;
  while (band < 3 && reading > POT_BOUND[band] + POT_HYSTERESIS) band++;
  while (band > 0 && reading < POT_BOUND[band - 1] - POT_HYSTERESIS) band--;

  if (band == potBand) return;
  potBand = band;

  if (band == 0) stopArm(POT_ARM);
  else           startArm(POT_ARM, (State)(band - 1));   // 1->LOW, 2->MED, 3->HIGH
}

// Any command from outside takes the object off its demo cycle.
void takeControl() { autoCycle = false; }

// ---------------------------------------------------------------- motion

void updateArms(float dt) {
  // Every state's clock advances every tick, whether or not an arm is currently using
  // it — that's what lets an arm joining a state mid-cycle land in step immediately
  // instead of resetting the state's cycle to zero for everyone already in it.
  for (uint8_t s = 0; s < STATE_COUNT; s++) statePhase[s] += STATE_RATE[s] * dt;

  float k = clampf(dt / STATE_BLEND_SEC, 0.0, 1.0);
  float maxStep = SLEW_DEG_PER_SEC * dt;

  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    State s = armState[i];
    bool moving = armRunning[i] && !holdCenter;

    // Ease amplitude toward this arm's state instead of jumping, so a swing-width
    // change reads as a transition rather than a snap. Phase isn't eased per arm at
    // all any more — it's read straight from that state's shared clock, which is
    // exactly what keeps every arm in a state moving as one.
    float wantAmp = moving ? STATE_AMP_DEG[s] : 0.0;   // stopping fades the swing out
    armAmp[i] += (wantAmp - armAmp[i]) * k;

    float target = moving
      ? CENTER_ANGLE[i] + sin(statePhase[s]) * armAmp[i]
      : CENTER_ANGLE[i];
    target = clampf(target, ANGLE_MIN, ANGLE_MAX);

    // slew limit so the arm never snaps
    float delta = target - armAngle[i];
    if (delta >  maxStep) delta =  maxStep;
    if (delta < -maxStep) delta = -maxStep;
    armAngle[i] += delta;

    if (servosAttached[i]) servos[i].write((int)(armAngle[i] + 0.5));

    // Deliberately never auto-releases at rest. A released servo goes limp, and
    // gravity pulls the arm's own weight off CENTER_ANGLE — exactly the "vertical at
    // rest" calibration this object depends on. Holding torque at idle costs a little
    // current and warmth in exchange for staying rigidly in place. Detach by hand with
    // E:0 (or the web page's controls) if you need the linkage to move freely, e.g.
    // while re-taping an arm.
  }
}

// ---------------------------------------------------------------- serial

void report() {
  Serial.print(F("S:"));
  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    if (i) Serial.print(';');
    Serial.print(STATE_NAME[armState[i]]);
    Serial.print(',');
    Serial.print(armRunning[i] ? 1 : 0);
    Serial.print(',');
    Serial.print((int)(armAngle[i] + 0.5));
  }
  Serial.println();
}

// Single characters typed into the Serial Monitor. Returns true if handled.
bool handleKey(char c) {
  switch (c) {
    case '1': takeControl(); startAll(ST_LOW);
              Serial.println(F("OK:key all LOW")); return true;
    case '2': takeControl(); startAll(ST_MED);
              Serial.println(F("OK:key all MED")); return true;
    case '3': takeControl(); startAll(ST_HIGH);
              Serial.println(F("OK:key all HIGH")); return true;
    case '0': takeControl(); stopAll();
              Serial.println(F("OK:key stop all")); return true;
    case 'a': autoCycle = true; holdCenter = false; lastSwitch = 0;
              Serial.println(F("OK:key demo cycle resumed")); return true;
    case 'c': takeControl(); holdCenter = true;
              for (uint8_t i = 0; i < ARM_COUNT; i++) { armRunning[i] = true; attachArm(i, true); }
              Serial.println(F("OK:key holding CENTER_ANGLE")); return true;
    default:  return false;
  }
}

void handleLine(char *s) {
  if (s[1] == '\0' && handleKey(s[0])) return;   // a bare single character

  char kind = toupper(s[0]);

  if (kind == '?') { report(); return; }

  // handshake — proof of life for the web page, and a visible blink on the board
  if (kind == 'H') {
    playPattern(PATTERN_HELLO);
    Serial.print(F("OK:HELLO social-battery ARMS="));
    Serial.println(ARM_COUNT);
    return;
  }

  // X -> stop everything,  X:<arm> -> stop one
  if (kind == 'X') {
    takeControl();
    if (s[1] == ':') {
      int idx = atoi(s + 2);
      if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }
      if (idx == POT_ARM) { Serial.println(F("ERR:arm is dial-controlled")); return; }
      stopArm(idx);
      Serial.print(F("OK:X ")); Serial.println(idx);
    } else {
      stopAll();
      Serial.println(F("OK:X all"));
    }
    return;
  }

  // T:<state> -> every arm,  T:<arm>:<state> -> one arm
  if (kind == 'T' && s[1] == ':') {
    takeControl();
    char *rest = s + 2;
    char *colon = strchr(rest, ':');

    if (colon) {                       // per-arm form
      *colon = '\0';
      int idx = atoi(rest);
      uint8_t st = parseState(colon + 1);
      if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }
      if (idx == POT_ARM)              { Serial.println(F("ERR:arm is dial-controlled")); return; }
      if (st >= STATE_COUNT)           { Serial.println(F("ERR:unknown state")); return; }
      startArm(idx, (State)st);
      Serial.print(F("OK:T ")); Serial.print(idx);
      Serial.print(' ');        Serial.println(STATE_NAME[st]);
      return;
    }

    uint8_t st = parseState(rest);     // all-arms form
    if (st >= STATE_COUNT) { Serial.println(F("ERR:unknown state")); return; }
    startAll((State)st);
    Serial.print(F("OK:T all ")); Serial.println(STATE_NAME[st]);
    return;
  }

  if (kind == 'E' && s[1] == ':') {
    for (uint8_t i = 0; i < ARM_COUNT; i++) attachArm(i, s[2] != '0');
    Serial.print(F("OK:E ")); Serial.println(s[2] != '0' ? 1 : 0);
    return;
  }

  Serial.println(F("ERR:unknown command"));
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) { line[lineLen] = '\0'; handleLine(line); lineLen = 0; }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    }
  }
}

// ---------------------------------------------------------------- arduino

void setup() {
  Serial.begin(115200);
  // Deliberately no `while (!Serial)` — that would hang the sketch whenever the board
  // runs without the Serial Monitor open, which is most of the time for a piece.

  pinMode(LED_PIN, OUTPUT);

  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    armAngle[i] = CENTER_ANGLE[i];
  }

  playPattern(PATTERN_HELLO);   // says "I booted" without blocking the loop

  Serial.print(F("OK:READY social-battery ARMS="));
  Serial.println(ARM_COUNT);
  Serial.println(F("keys: 1/2/3 = low/med/high, 0 = stop, a = demo cycle, c = hold centre"));

  lastTick = millis();
  lastSwitch = 0;               // start the demo cycle immediately
}

void loop() {
  readSerial();
  updateLed();
  readPot();

  unsigned long now = millis();

  // Standalone demo: walk every arm through the states until something takes over.
  if (autoCycle && (lastSwitch == 0 || now - lastSwitch >= DWELL_MS)) {
    if (lastSwitch != 0) cycleState = (cycleState + 1) % STATE_COUNT;
    lastSwitch = now;
    startAll((State)cycleState);
    Serial.print(F("demo -> ")); Serial.println(STATE_NAME[cycleState]);
  }

  float dt = (now - lastTick) / 1000.0;
  if (dt >= 0.02) {            // 50 Hz motion update
    lastTick = now;
    updateArms(dt);
  }

  if (now - lastReport >= 100) {   // 10 Hz telemetry
    lastReport = now;
    report();
  }
}
