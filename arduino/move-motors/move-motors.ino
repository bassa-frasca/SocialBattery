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

  The red arm is a special case: a MOTORIZED FADER is its controller, entirely on its
  own pins and its own loop — it neither drives the servo arms nor is driven by them.
  Today it is HAND-ONLY: move it and it captures your gesture, lets go and it mirrors
  that position, ping-ponging between the two at a speed set by which level you left
  it on. Software cannot yet command it — T:<arm>/X:<arm>/F: addressing the fader are
  not implemented (see the open items below); the only thing that moves it is your hand.

  Red never joins the standalone demo, and "every arm" commands never touch it — it
  isn't in ARM_COUNT at all, so it's simply not addressable by arm index.

  Board: Arduino MKR WiFi 1010. Serial is the native USB port, 115200 baud.

  ---------------------------------------------------------------- protocol
  One ASCII command per line, '\n' terminated. All of it addresses the two servo arms
  (0=blue, 1=yellow) only — the fader has no serial commands yet:

    H                  handshake: blink the LED pattern and identify the board
    T:<arm>:<state>    start ONE servo arm in that state, e.g. T:0:HIGH
    T:<state>          start EVERY servo arm in that state
    X:<arm>            stop one servo arm
    X                  stop every servo arm
    V                  print the live amplitude/speed table
    V:<state>:<amp>:<rate>   retune a state live, e.g. V:MED:24:0.7
    E:<0|1>            detach / attach the servos by hand
    ?                  send one status line now

  Replies:
    OK:HELLO social-battery ARMS=<n>                     answer to H
    S:<state>,<run>,<angle>;<state>,<run>,<angle>;...    one group per servo arm, 10 Hz
    FD:<pos>,<band>,<seeking>                            fader telemetry, 10 Hz — read-only,
                                                          reports what your hand is doing
    OK:<echo>  /  ERR:<reason>

  ---------------------------------------------------------------- open items
  The header comment used to claim T:<arm>/X:<arm>/F:<pos> could drive the fader in
  software — they were never implemented (no case for them in handleLine()), so that
  was aspirational, not real. Actually wiring that up is its own piece of work: a
  target-seeking mode for faderUpdate() that the hand-detection logic can still
  interrupt. Left undone deliberately rather than bolted on without being asked.

  ---------------------------------------------------------------- serial monitor
  Typing the protocol by hand is tedious, so single characters work too:

    1 / 2 / 3   all arms to low / medium / high
    0           stop all arms
    a           resume the automatic demo cycle
    c           hold every arm at CENTER_ANGLE, for setting the resting pose

  Digits and lowercase letters were chosen so they cannot collide with the protocol.

  ---------------------------------------------------------------- wiring
  Two servo signals -> D2 (blue), D3 (yellow) — see SERVO_PIN below.
  Servo power from an EXTERNAL 5V supply, its ground tied to the board's ground — do not
  run servos off the MKR's own regulator.

  NeoPixel strip (8 pixels, red's level indicator) -> D1 data.

  Motorized fader (100mm, 6 wires) + HW-354 driver, driving red:
    POT. SLIDER  (green)  -> A1            the wiper, position feedback
    VCC SLIDER   (red)    -> MKR VCC 3.3V  NEVER 5V, the analog pins are not 5V tolerant
    GND SLIDER   (grey)   -> MKR GND
    VCC MOTOR    (red)    -> driver Motor A / OUT1   (swap with OUT2 to reverse)
    GND MOTOR    (black)  -> driver Motor A / OUT2
    TOUCH SLIDER (orange) -> unconnected, capacitive strip not used yet
    driver IN1 -> D4,  IN2 -> D5.  D4 is also PIN_SPI_SS on this board's variant —
    a known-risky alias, documented rather than avoided, since it's what's actually
    wired. If the fader ever misbehaves in a way power/wiring doesn't explain,
    suspect this pin first and try moving IN1 elsewhere (D7 was the previous home).
    driver VCC/GND -> EXTERNAL 5V supply, ground tied to MKR GND. Running the driver
    off the MKR's 3.3V pin makes the fader crawl and browns the board off USB.

  Avoid D8/D9/D10 (the NINA WiFi SPI bus) for anything.
*/

#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ---------------------------------------------------------------- pins
//
// One entry per arm. Add pins here and the whole sketch adapts — per-arm state,
// telemetry and the arm count reported to the web page all size themselves from it.
//
// Set to 1 to bring the servo arms back. With them off, no pulses are ever sent and
// the servos draw nothing — which leaves the whole supply for the fader motor.
#define SERVOS_ENABLED 1

// Release a stopped arm once it has settled, instead of holding it there.
//
// Holding torque at idle keeps the arm rigidly on CENTER_ANGLE, which is what the
// object's "vertical at rest" look depends on — but it means a stopped servo is
// pushing continuously, and if the arm is fighting anything mechanical that is a
// permanent stall: full current, no movement, all of it heat. A servo that gets hot
// while doing nothing is telling you it cannot reach where it is being sent.
//
// Set to 1 to go back to holding.
#define HOLD_AT_REST 0
#define RELEASE_AFTER_MS 900   // settle time before letting go

const uint8_t SERVO_PIN[] = { 2, 3 };
//
// Arm order follows this array: index 0 is the blue square on the longest rod at the
// back (D2), 1 the yellow wedge in the middle (D3). Red has no servo — it's the
// motorized fader instead (see the FADER section below), so it isn't in this array
// and doesn't count toward ARM_COUNT.
//
// Avoid pins 8, 9 and 10 on the MKR WiFi 1010 — they are the SPI bus to the onboard
// NINA WiFi module. A servo there works only until something switches the radio on.

const uint8_t MAX_ARMS  = 3;   // one spare slot; CENTER_ANGLE etc. keep 3 entries
const uint8_t ARM_COUNT = sizeof(SERVO_PIN) / sizeof(SERVO_PIN[0]);
const uint8_t LED_PIN   = LED_BUILTIN;   // pin 6 on the MKR boards

// The SAMD Servo library drives any digital pin from a hardware timer rather than from
// analogWrite, so the plain digital pins D2 and D3 are both fine here.

// ---------------------------------------------------------------- tuning

// Resting pose — the middle of each arm's swing. State changes how an arm MOVES, not
// where it sits. Three entries, so the others are ready when you add them.
// Arm 1 is confirmed vertical at 96 on the real object. Arm 0 was inherited at 60,
// which put HIGH's sweep at 20-100 degrees — far enough down that the linkage stalled
// against its stop and the servo cooked. Starting it at the known-good 96 until it is
// measured; use C:<arm>:<angle> to find the true vertical and paste it back here.
const int CENTER_ANGLE[MAX_ARMS] = { 96, 96, 93 };

// Hard clamp on every movement. Keep inside whatever the linkage can physically reach.
const int ANGLE_MIN = 10;
const int ANGLE_MAX = 170;

enum State { ST_LOW = 0, ST_MED = 1, ST_HIGH = 2, STATE_COUNT = 3 };
const char *STATE_NAME[STATE_COUNT] = { "LOW", "MED", "HIGH" };

// The whole design lives in these two tables. Amplitude is degrees either side of
// centre — the swing is symmetrical, so the arm travels this many degrees each way
// from CENTER_ANGLE; rate is radians per second.
//                                          LOW    MED    HIGH
// Live-tunable so a state can be dialled in against the real object without a
// re-upload — see the V: command. Whatever you settle on, paste it back here.
// Equal quarters, the same division the fader uses. ARM_REACH is how far an arm can
// swing either side of centre; Low/Medium/High take two, three and four quarters of
// it, so each step up widens the swing by the same amount.
#define ARM_REACH 40.0
float stateAmp[STATE_COUNT]  = { ARM_REACH * 0.50, ARM_REACH * 0.75, ARM_REACH };
float stateRate[STATE_COUNT] = { 0.35,  0.90,  1.10 };  // radians per second

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
float armAngle[MAX_ARMS];
unsigned long restSince[MAX_ARMS] = { 0, 0, 0 };   // when an arm arrived at rest       // what the servo is actually holding
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
  if (!SERVOS_ENABLED) return;   // arms parked: never attach, never pulse
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
  if (!armRunning[i]) armAmp[i] = stateAmp[s];
  armState[i] = s;
  armRunning[i] = true;
  holdCenter = false;
  attachArm(i, true);
}

void startAll(State s) { for (uint8_t i = 0; i < ARM_COUNT; i++) startArm(i, s); }

// Any command from outside takes the object off its standalone demo cycle.
void takeControl() { autoCycle = false; }

void stopArm(uint8_t i) { armRunning[i] = false; }   // eases home and holds there, powered
void stopAll() {
  for (uint8_t i = 0; i < ARM_COUNT; i++) stopArm(i);
  holdCenter = false;
}

// ---------------------------------------------------------------- motion

void updateArms(float dt) {
  // Every state's clock advances every tick, whether or not an arm is currently using
  // it — that's what lets an arm joining a state mid-cycle land in step immediately
  // instead of resetting the state's cycle to zero for everyone already in it.
  for (uint8_t s = 0; s < STATE_COUNT; s++) statePhase[s] += stateRate[s] * dt;

  float k = clampf(dt / STATE_BLEND_SEC, 0.0, 1.0);
  float maxStep = SLEW_DEG_PER_SEC * dt;

  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    State s = armState[i];
    bool moving = armRunning[i] && !holdCenter;

    // Ease amplitude toward this arm's state instead of jumping, so a swing-width
    // change reads as a transition rather than a snap. Phase isn't eased per arm at
    // all any more — it's read straight from that state's shared clock, which is
    // exactly what keeps every arm in a state moving as one.
    float wantAmp = moving ? stateAmp[s] : 0.0;   // stopping fades the swing out
    armAmp[i] += (wantAmp - armAmp[i]) * k;

    // Ping-pong, not a sine — the same shape the fader makes. A sine spends most of
    // its time near the ends and eases through the middle; a ping-pong crosses at a
    // constant rate and turns sharply, which is what the motorized fader physically
    // does and what makes the two read as one object.
    //
    // A triangle wave from the state's shared clock: phase runs 0..2PI as before, the
    // first half sweeping one way and the second half back.
    float cycle = fmod(statePhase[s], TWO_PI);
    if (cycle < 0) cycle += TWO_PI;
    float tri = (cycle < PI) ? (cycle / PI) * 2.0 - 1.0     // -1 -> +1
                             : 1.0 - ((cycle - PI) / PI) * 2.0;  // +1 -> -1

    float target = moving
      ? CENTER_ANGLE[i] + tri * armAmp[i]
      : CENTER_ANGLE[i];
    target = clampf(target, ANGLE_MIN, ANGLE_MAX);

    // slew limit so the arm never snaps
    float delta = target - armAngle[i];
    if (delta >  maxStep) delta =  maxStep;
    if (delta < -maxStep) delta = -maxStep;
    armAngle[i] += delta;

    if (servosAttached[i]) servos[i].write((int)(armAngle[i] + 0.5));

    // Once a stopped arm has arrived, let it go — nothing to hold it against.
    if (!HOLD_AT_REST && !armRunning[i] && servosAttached[i]
        && fabs(armAngle[i] - CENTER_ANGLE[i]) <= 1.0) {
      if (restSince[i] == 0) restSince[i] = millis();
      else if (millis() - restSince[i] >= RELEASE_AFTER_MS) attachArm(i, false);
    } else if (armRunning[i]) {
      restSince[i] = 0;
    }

    // Deliberately never auto-releases at rest. A released servo goes limp, and
    // gravity pulls the arm's own weight off CENTER_ANGLE — exactly the "vertical at
    // rest" calibration this object depends on. Holding torque at idle costs a little
    // current and warmth in exchange for staying rigidly in place. Detach by hand with
    // E:0 (or the web page's controls) if you need the linkage to move freely, e.g.
    // while re-taping an arm.
  }
}

// ================================================================== FADER
//
// A motorized fader with its own NeoPixel strip, on its own pins. It reads,
// captures and mimics a hand-set gesture entirely on its own - it neither
// drives the servo arms nor is driven by them. The two subsystems share only
// the board and the serial port.
// ==========================================================================

// ---- motorized fader + NeoPixel (independent of the arms) -------------------
// D4 is also PIN_SPI_SS on this board's variant — a documented reason to avoid it
// (see the wiring note in the header comment), but it's what's actually wired right
// now, so that's what this reads. If the fader misbehaves in a way that isn't
// explained by power/wiring, this alias is the first thing to suspect — move IN1 to
// a different free PWM pin (D2/D3 are taken by the servos; D7 was the previous home)
// and see if the symptom follows the pin.
#define MOTOR_IN1 4    // HW-354 IN1 - shares a pin with PIN_SPI_SS, see above
#define MOTOR_IN2 5    // HW-354 IN2 (Motor A) - direction + speed
#define SLIDER_PIN A1  // the fader's wiper
#define STRIP_PIN 1    // NeoPixel data

// ---- Travel limits, measured on the bench ----------------------------------
#define SLIDER_MIN 336      // reading at the low mechanical stop
#define SLIDER_MAX 753      // reading at the high stop
#define SLIDER_CENTER ((SLIDER_MIN + SLIDER_MAX) / 2)   // 544

// ---- The levels ------------------------------------------------------------
// The faderLevel is how far the slider sits FROM THE CENTRE, in either direction - so
// the scale is symmetric, and pushing away from the middle in either direction
// raises the intensity. That is what makes the mirror swing work: a position and
// its reflection are always the same faderLevel.
//
//     336 / 753  ->  208 from centre  ->  HIGH
//     386 / 703  ->  158              ->  MEDIUM
//     436 / 653  ->  108              ->  LOW
//     500        ->   44              ->  OFF
//
// Boundaries sit midway between the measured points.
#define DIST_OFF_LOW   77
#define DIST_LOW_MED  134
#define DIST_MED_HIGH 184

// A boundary has to be cleared by this much before the faderLevel actually changes, so
// a reading sitting right on a mark cannot flicker between two levels.
#define BAND_HYSTERESIS 8

// ---- NeoPixel --------------------------------------------------------------
// Eight pixels. The lit pair moves OUTWARD from the middle as the faderLevel rises,
// so the strip reads as the battery opening up.
//
//     off      nothing
//     low      pixels 4,5            the middle pair
//     medium   pixels 3,4,5,6        widening
//     high     pixels 1..8           the whole strip
//
// Numbering below is 0-based, so your 1..8 become 0..7.
#define STRIP_COUNT 8
#define STRIP_BRIGHTNESS 90

// How hard each faderLevel is driven. The swing's ENDS come from the mirror - where you
// left the slider, and its reflection - so the gesture is yours; the faderLevel only says
// how energetically it comes back.
//                            Off  Low  Medium  High
const int SWING_SPEED[4] = {   0,  230,   243,   255 };
const char* LEVEL_NAME[4] = { "OFF", "LOW", "MEDIUM", "HIGH" };

// Below this the two ends are too close together to be worth swinging between
#define MIN_SWING 60

// Overshoot control. At full duty the fader sails past the target, which then reads
// as the gap growing. Ease off over the last stretch instead of arriving flat out.
// The floor is the lowest duty that still reliably breaks friction - not measured,
// so it is set conservatively high.
#define RAMP_ZONE 120
#define MIN_DUTY 215        // 154 did not move it at all; 190 still crawled

// Friction varies along the track and between faders, and the duty that actually
// breaks it has never been measured properly. So rather than trust a guess: if the
// slider stops making progress while we are driving it, wind the duty up until it
// moves again, and report the value that worked.
#define STALL_MS 200        // no progress for this long while driving = stuck
#define STALL_STEP 12       // how much to add each time
#define STALL_NOISE 2       // counts of change that do not count as progress
#define BOOST_MAX 60        // never wind past this - beyond it, assume a hand
#define HOLD_MS 500         // pushing at full boost this long without moving = held

int driveSpeed(int gap, int full) {
  // Nothing to taper if the level already runs at or below the friction floor -
  // and tapering anyway inverted the ramp, making the slowest level speed UP as it
  // approached its target.
  if (full <= MIN_DUTY) return full;
  if (gap >= RAMP_ZONE) return full;
  int duty = MIN_DUTY + (long)(full - MIN_DUTY) * gap / RAMP_ZONE;
  if (duty < MIN_DUTY) duty = MIN_DUTY;
  if (duty > full) duty = full;      // a taper must never exceed the cruise speed
  return duty;
}

// ---- Control Parameters ----------------------------------------------------
#define HOMING_SPEED 220               // Speed used to drive to the Off mark
#define FADER_MARGIN 10                // Acceptable position error, ~1% of range
#define PRINT_INTERVAL 200             // How often to print debug values (ms)
#define FADER_MOVE_TIMEOUT 8000              // Give up on an end rather than push into
                                       // a mechanical stop forever
#define FADER_SAMPLES 8                      // analogRead samples averaged per reading

// ---- Hand detection --------------------------------------------------------
// While driving, the slider should get steadily CLOSER to its target. If the gap
// instead grows by more than this, something is pushing back - you.
// Catching a hand while the motor is driving.
//
// The old test - "the gap to the target grew" - needed the leg to have closed 45
// counts before it would arm, so grabbing early in a leg was simply invisible. And
// it needed a big push to register. This looks at DIRECTION instead: the slider
// should be moving the way we are driving it, and if it moves the other way, that is
// a hand. Far more sensitive, and it works from the first moment of a leg.
//
// The only thing it has to forgive is the instant after a turn, when the fader is
// still carrying momentum the old way - hence a short grace period rather than a
// distance threshold.
#define GRACE_MS 150        // ignore direction right after a turn
#define REVERSE_MARGIN 12   // counts moved against the drive = a hand
#define GRAB_MARGIN 45      // legacy gap test, kept as a slower backstop
#define SETTLE_MOVE 8      // Counts of change that still count as "hand moving"
#define SETTLE_MS 400      // Hand still this long = you let go

// Function Declarations
void motorCoast();
void motorStop();
void motorForward(int speed);
void motorBackward(int speed);

// ---- State -----------------------------------------------------------------
int faderLevel = 0;       // 0 Off, 1 Low, 2 Medium, 3 High
int pointA = 0;      // low end of the current swing
int pointB = 0;      // high end
int targetVal = 0;   // whichever end we are heading for right now
int lastSliderReading = 0;   // set once per faderUpdate(), so reportFader() below reads
                              // it rather than sampling the ADC a second time per tick
unsigned long manualDriveUntil = 0;   // faderUpdate() stands down until millis() passes
                                       // this, while a raw D: test command holds the motor

enum FaderMode { FD_HOMING, FD_SWINGING, FD_GRABBED, FD_IDLE };
FaderMode faderMode = FD_HOMING;

int bestGap = 0;     // Closest we have got to the target on this leg
int legStartGap = 0; // Gap when this leg began
bool grabArmed = false;  // Grab detection only counts once the leg is making progress
int prevSlider = 0;      // Last reading, for working out which way it is moving
int against = 0;         // Counts moved against the drive on this leg
unsigned long holdTime = 0;
int stallRef = 0;        // Reading the stall timer measures progress against
unsigned long stallTime = 0;
int boost = 0;           // Extra duty added to break friction when stuck
int boostReported = 0;   // Highest boost we have mentioned, so it is logged once
int settleRef = 0;   // Reading the settle timer is measured against
int idleRef = 0;     // Reading we watch for a hand while resting

unsigned long faderLastPrint = 0;
unsigned long moveStart = 0;
unsigned long settleTime = 0;

const char* faderPhaseName() {
  if (faderMode == FD_HOMING)  return "FD_HOMING  ";
  if (faderMode == FD_IDLE)    return "RESTING ";
  if (faderMode == FD_GRABBED) return "HAND    ";
  return "FD_SWINGING";
}

// Average several samples. A single analogRead picks up motor noise, which would
// otherwise look like movement - or like a hand on the slider. Measured +/-1
// count at 8 samples, against +/-15 with a single read.
int readSlider() {
  long total = 0;
  for (int i = 0; i < FADER_SAMPLES; i++) {
    total += analogRead(SLIDER_PIN);
  }
  return (int)(total / FADER_SAMPLES);
}

// Which faderLevel a reading falls in, from its distance either side of centre.
// `current` is the faderLevel already showing; a reading has to clear the boundary by
// BAND_HYSTERESIS to move off it, so noise on a mark cannot flicker the faderLevel.
int levelFor(int reading, int current) {
  int dist = abs(reading - SLIDER_CENTER);
  const int edge[3] = { DIST_OFF_LOW, DIST_LOW_MED, DIST_MED_HIGH };

  int lv = current;
  while (lv < 3 && dist > edge[lv] + BAND_HYSTERESIS) lv++;
  while (lv > 0 && dist < edge[lv - 1] - BAND_HYSTERESIS) lv--;
  return lv;
}

// ---- NeoPixel --------------------------------------------------------------------
Adafruit_NeoPixel strip(STRIP_COUNT, STRIP_PIN, NEO_GRB + NEO_KHZ800);

// Which pixels each faderLevel lights, as a bitmask over 0..7. The fill grows OUTWARD
// from the middle pair, so the strip reads as a faderLevel rising rather than a pattern
// changing - each faderLevel keeps everything the one below it lit.
//
//     off      nothing
//     low      4,5              the middle pair
//     medium   3,4,5,6          widening
//     high     1,2,3,4,5,6,7,8  the whole strip
//
// Bit 0 is pixel 1, so the masks below read right-to-left.
const uint8_t STRIP_MASK[4] = { 0b00000000, 0b00011000, 0b00111100, 0b11111111 };

// Colour per faderLevel. The fill grows outward AND heats up as the faderLevel rises.
const uint32_t STRIP_COLOUR[4] = {
  0x000000,   // off
  0xFFC400,   // low    - yellow
  0xFF6A00,   // medium - orange
  0xFF1FA0,   // high   - magenta
};

int shownLevel = -1;   // so the strip is only rewritten when it actually changes

void showLevel(int lv) {
  if (lv == shownLevel) return;
  shownLevel = lv;

  uint8_t mask = STRIP_MASK[lv];
  uint32_t c = STRIP_COLOUR[lv];
  for (int i = 0; i < STRIP_COUNT; i++) {
    strip.setPixelColor(i, (mask & (1 << i)) ? c : 0);
  }
  strip.show();
}

// Capture: the slider has been left somewhere, so mirror that position and swing
// between the two. The mirror of x in [min, max] is (min + max - x).
void detectFrom(int x) {
  pointA = constrain(x, SLIDER_MIN, SLIDER_MAX);
  pointB = constrain(SLIDER_MIN + SLIDER_MAX - pointA, SLIDER_MIN, SLIDER_MAX);

  faderLevel = levelFor(pointA, faderLevel);

  Serial.print("DETECTED x = ");
  Serial.print(pointA);
  Serial.print("  ->  MODE ");
  Serial.print(LEVEL_NAME[faderLevel]);
  Serial.print("  swinging ");
  Serial.print(pointA);
  Serial.print(" <-> ");
  Serial.print(pointB);
  Serial.print("  at speed ");
  Serial.println(SWING_SPEED[faderLevel]);

  if (faderLevel == 0) {
    // Off just rests where you left it. Centring happens once, at power-up, and
    // never again - dragging it back to the middle every time would fight you.
    Serial.println("  (OFF - resting here)");
    motorCoast();
    idleRef = pointA;
    targetVal = pointA;
    faderMode = FD_IDLE;
    return;
  }

  if (abs(pointB - pointA) < MIN_SWING) {
    // x sat near the middle, so x and its mirror nearly coincide. Twitching across
    // a few counts reads as a fault, so rest here and wait for a hand to give us
    // something to work with.
    Serial.println("  (too close to the centre to swing - resting, move the slider)");
    motorCoast();
    idleRef = pointA;
    targetVal = pointA;
    faderMode = FD_IDLE;
    return;
  }

  targetVal = pointB;
  bestGap = abs(targetVal - pointA);
  legStartGap = bestGap;
  grabArmed = false;
  boost = 0;
  stallRef = -999;
  stallTime = millis();
  holdTime = millis();
  against = 0;
  prevSlider = pointA;
  moveStart = millis();
  faderMode = FD_SWINGING;
}

// Flip to the other end of the swing and restart this leg
void swapTarget(int sliderVal) {
  targetVal = (targetVal == pointA) ? pointB : pointA;
  bestGap = abs(targetVal - sliderVal);
  legStartGap = bestGap;
  grabArmed = false;
  boost = 0;
  stallRef = -999;
  stallTime = millis();
  holdTime = millis();
  against = 0;
  prevSlider = sliderVal;
  moveStart = millis();
}

void handDetected(int sliderVal) {
  Serial.println("HAND DETECTED - motor released, set it where you like");
  motorCoast();
  faderMode = FD_GRABBED;
  settleRef = sliderVal;
  settleTime = millis();
}

void faderSetup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorCoast();

  strip.begin();
  strip.setBrightness(STRIP_BRIGHTNESS);
  strip.clear();
  strip.show();

  Serial.print(F("fader: travel ")); Serial.print(SLIDER_MIN);
  Serial.print(F(" - ")); Serial.print(SLIDER_MAX);
  Serial.print(F(", centre ")); Serial.println(SLIDER_CENTER);
  Serial.print(F("fader: homing to ")); Serial.println(SLIDER_CENTER);
  moveStart = millis();
  faderMode = FD_HOMING;
}

void faderUpdate() {
  // A raw D: test command is in control — stand down entirely rather than merely
  // switching to FD_IDLE, whose own per-loop behaviour is motorCoast(), which would
  // silently cancel whatever D: just commanded on the very next loop() iteration.
  if (millis() < manualDriveUntil) return;

  int sliderVal = readSlider();
  lastSliderReading = sliderVal;

  // The strip shows the faderLevel you SET, and holds it.
  //
  // Not the faderLevel of wherever the slider happens to be this millisecond: during a
  // swing the fader sweeps across every band on its way between the two ends, so
  // following the live reading made the lights flicker through yellow, orange and
  // magenta continuously. The colour only changes when you set a new faderLevel.
  showLevel(faderLevel < 0 ? 0 : faderLevel);

  // Print status on an interval. Never use delay() here - the loop has to keep
  // sampling, or it misses the movement it is supposed to detect.
  if (millis() - faderLastPrint >= PRINT_INTERVAL) {
    faderLastPrint = millis();

    unsigned long ms = millis();
    Serial.print("t=");
    Serial.print(ms / 1000);
    Serial.print(".");
    unsigned long frac = ms % 1000;
    if (frac < 100) Serial.print("0");
    if (frac < 10)  Serial.print("0");
    Serial.print(frac);
    Serial.print("s  ");

    Serial.print(faderPhaseName());
    Serial.print("  slider: ");
    Serial.print(sliderVal);
    Serial.print("  faderLevel: ");
    Serial.print(LEVEL_NAME[faderLevel]);
    Serial.print("  target: ");
    if (faderMode == FD_HOMING) Serial.println(SLIDER_CENTER);
    else if (faderMode == FD_SWINGING) Serial.println(targetVal);
    else Serial.println("-");
  }

  switch (faderMode) {

    // ---- Drive to the Off mark on power-up ----
    case FD_HOMING: {
      bool timedOut = (millis() - moveStart >= FADER_MOVE_TIMEOUT);

      if (abs(SLIDER_CENTER - sliderVal) <= FADER_MARGIN || timedOut) {
        motorStop();
        if (timedOut) {
          Serial.print("FD_HOMING gave up at ");
          Serial.println(sliderVal);
        } else {
          Serial.print("AT OFF MARK ");
          Serial.println(sliderVal);
        }
        // Settle here rather than calling detectFrom() again: at the centre the faderLevel
        // is OFF, and OFF sends us back to FD_HOMING, which would loop forever.
        motorCoast();
        faderLevel = levelFor(sliderVal, faderLevel);
        idleRef = sliderVal;
        targetVal = sliderVal;
        faderMode = FD_IDLE;
        Serial.print("  resting at ");
        Serial.print(sliderVal);
        Serial.print(", faderLevel ");
        Serial.println(LEVEL_NAME[faderLevel]);
      } else if (sliderVal > SLIDER_CENTER) {
        motorBackward(HOMING_SPEED);
      } else {
        motorForward(HOMING_SPEED);
      }
      break;
    }

    // ---- Resting: motor off, waiting for a hand ----
    case FD_IDLE: {
      motorCoast();
      if (abs(sliderVal - idleRef) > SETTLE_MOVE) {
        Serial.println("HAND DETECTED - set it where you like");
        faderMode = FD_GRABBED;
        settleRef = sliderVal;
        settleTime = millis();
      }
      break;
    }

    // ---- Your hand is on it: motor off, wait for you to finish ----
    case FD_GRABBED: {
      motorCoast();
      if (abs(sliderVal - settleRef) > SETTLE_MOVE) {
        settleRef = sliderVal;
        settleTime = millis();
      } else if (millis() - settleTime >= SETTLE_MS) {
        // Still for long enough - that is where you wanted it
        detectFrom(sliderVal);
      }
      break;
    }

    // ---- Swinging across the selected faderLevel's range ----
    case FD_SWINGING: {
      int gap = abs(targetVal - sliderVal);
      int dir = (targetVal > sliderVal) ? 1 : -1;      // the way we are driving
      int delta = sliderVal - prevSlider;              // the way it actually went
      prevSlider = sliderVal;

      bool pastGrace = (millis() - moveStart >= GRACE_MS);

      // Moving against the drive is a hand - nothing else pushes back.
      if (delta * dir > 0) {
        against = 0;                    // making progress, forget any wobble
      } else if (delta * dir < 0) {
        against += -delta * dir;
        if (pastGrace && against > REVERSE_MARGIN) {
          handDetected(sliderVal);
          break;
        }
      }

      // Safety net - if an end cannot be reached, swing back rather than grind
      if (millis() - moveStart >= FADER_MOVE_TIMEOUT) {
        Serial.print("GAVE UP at ");
        Serial.print(sliderVal);
        Serial.print(" chasing ");
        Serial.println(targetVal);
        motorStop();
        swapTarget(sliderVal);
        break;
      }

      if (gap > FADER_MARGIN) {
        if (abs(sliderVal - stallRef) > STALL_NOISE) {
          // moving again
          stallRef = sliderVal;
          stallTime = millis();
          holdTime = millis();
          if (boost > boostReported) {
            Serial.print("  broke friction at duty +");
            Serial.println(boost);
            boostReported = boost;
          }
          boost = 0;
        } else if (millis() - stallTime >= STALL_MS) {
          stallTime = millis();
          // Wind up to break friction - but only so far. Past BOOST_MAX the thing
          // stopping it is not friction, it is a hand holding it still, and shoving
          // harder is precisely the wrong answer.
          if (boost < BOOST_MAX) {
            boost += STALL_STEP;
          } else if (pastGrace && millis() - holdTime >= HOLD_MS) {
            Serial.println("  (held still against full drive - treating as a hand)");
            handDetected(sliderVal);
            break;
          }
        }
      }

      int duty = driveSpeed(gap, SWING_SPEED[faderLevel]) + boost;
      if (duty > 255) duty = 255;

      if (sliderVal > targetVal + FADER_MARGIN) {
        motorBackward(duty);
      } else if (sliderVal < targetVal - FADER_MARGIN) {
        motorForward(duty);
      } else {
        // Reached this end - turn around immediately, no dwell
        motorStop();
        swapTarget(sliderVal);
      }
      break;
    }
  }
}

// Motor Control Functions
//
// The HW-354 board has no enable pin: direction AND speed both come from
// IN1/IN2. Hold one input LOW and PWM the other to drive at that speed.
//
//   IN1   IN2   result
//   LOW   LOW   coast  - free to move by hand
//   PWM   LOW   forward at duty
//   LOW   PWM   backward at duty
//   HIGH  HIGH  brake  - resists movement, holds position
//
// Everything below uses analogWrite() rather than digitalWrite(). On the
// MKR 1010 (SAMD21) analogWrite() re-muxes the pin to a timer peripheral, and a
// later digitalWrite() on that same pin is ignored until pinMode() is called
// again. Staying on analogWrite() the whole way avoids that trap.

void motorCoast() {
  // Both inputs LOW - motor unpowered, slider moves freely by hand
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, 0);
}

void motorStop() {
  // Brake motor by setting both inputs HIGH (full duty)
  analogWrite(MOTOR_IN1, 255);
  analogWrite(MOTOR_IN2, 255);
}

void motorForward(int speed) {
  // Idle pin first so the two inputs are never briefly both driven
  analogWrite(MOTOR_IN2, 0);
  analogWrite(MOTOR_IN1, speed);
}

void motorBackward(int speed) {
  // Idle pin first so the two inputs are never briefly both driven
  analogWrite(MOTOR_IN1, 0);
  analogWrite(MOTOR_IN2, speed);
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

// FD:<pos>,<band>,<seeking> — the fader's own line, sent alongside S: at 10 Hz. "pos"
// is the last slider reading faderUpdate() took (not a fresh ADC sample — the fader
// runs its own loop, this just reports what it already knows). "seeking" is 1 while
// the motor is actively driving toward a target (homing or swinging) and 0 while it's
// coasting (idle or a hand is on it) — the web page uses that to know when to trust
// this position as steady rather than mid-swing.
void reportFader() {
  bool seeking = (faderMode == FD_HOMING || faderMode == FD_SWINGING);
  Serial.print(F("FD:"));
  Serial.print(lastSliderReading);
  Serial.print(',');
  Serial.print(faderLevel);
  Serial.print(',');
  Serial.println(seeking ? 1 : 0);
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

  if (kind == '?') { report(); reportFader(); return; }

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
      // The fader arm is commandable now: set the state AND drive the slider to match,
      // so the physical control never disagrees with what the software thinks red is.
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

  // C:<arm>:<angle> — hold one arm at an angle, so the resting vertical can be found
  // by eye rather than guessed. It stays there until you stop it or set a state.
  if (kind == 'C' && s[1] == ':') {
    char *p1 = strchr(s + 2, ':');
    if (!p1) { Serial.println(F("ERR:C needs arm and angle")); return; }
    *p1 = '\0';
    int idx = atoi(s + 2);
    int ang = atoi(p1 + 1);
    if (idx < 0 || idx >= ARM_COUNT) { Serial.println(F("ERR:arm out of range")); return; }
    ang = (int)clampf(ang, ANGLE_MIN, ANGLE_MAX);
    takeControl();
    armRunning[idx] = false;
    restSince[idx] = 0;
    attachArm(idx, true);
    armAngle[idx] = ang;
    servos[idx].write(ang);
    Serial.print(F("OK:C ")); Serial.print(idx);
    Serial.print(F(" holding ")); Serial.println(ang);
    return;
  }

  // V:<state>:<amp>:<rate> — retune a state live, no re-upload. e.g. V:MED:24:0.7
  // V alone prints the current table, ready to paste back into the sketch.
  if (kind == 'V') {
    if (s[1] == '\0') {
      for (uint8_t i = 0; i < STATE_COUNT; i++) {
        Serial.print(F("V:")); Serial.print(STATE_NAME[i]);
        Serial.print(F(" amp=")); Serial.print(stateAmp[i], 1);
        Serial.print(F(" rate=")); Serial.println(stateRate[i], 2);
      }
      return;
    }
    if (s[1] != ':') { Serial.println(F("ERR:use V or V:<state>:<amp>:<rate>")); return; }

    char *p1 = strchr(s + 2, ':');
    if (!p1) { Serial.println(F("ERR:V needs state, amp and rate")); return; }
    *p1 = '\0';
    char *p2 = strchr(p1 + 1, ':');
    if (!p2) { Serial.println(F("ERR:V needs state, amp and rate")); return; }
    *p2 = '\0';

    uint8_t st = parseState(s + 2);
    if (st >= STATE_COUNT) { Serial.println(F("ERR:unknown state")); return; }

    float amp  = atof(p1 + 1);
    float rate = atof(p2 + 1);
    // Clamp rather than reject: a typo should not fling an arm into its end stop.
    if (amp  < 0)    amp  = 0;
    if (amp  > 60)   amp  = 60;
    if (rate < 0.01) rate = 0.01;
    if (rate > 6.0)  rate = 6.0;

    stateAmp[st]  = amp;
    stateRate[st] = rate;

    // Arms already running in this state pick the change up through the usual blend,
    // so you hear and see the difference immediately rather than after a restart.
    Serial.print(F("OK:V ")); Serial.print(STATE_NAME[st]);
    Serial.print(F(" amp=")); Serial.print(amp, 1);
    Serial.print(F(" rate=")); Serial.println(rate, 2);
    return;
  }

  // D:<duty> — drive the fader motor directly, bypassing every bit of logic, for 2
  // seconds (renewed by each new D: command). Positive drives forward, negative
  // backward, 0 coasts, 999 brakes. Purely a wiring/supply probe: if this does
  // nothing, the fault is not in the control code.
  if (kind == 'D' && s[1] == ':') {
    int duty = atoi(s + 2);
    manualDriveUntil = millis() + 2000;
    if (duty == 999)   motorStop();      // brake: both inputs HIGH, windings shorted
    else if (duty > 0) motorForward(duty > 255 ? 255 : duty);
    else if (duty < 0) motorBackward(-duty > 255 ? 255 : -duty);
    else               motorCoast();
    Serial.print(F("OK:D ")); Serial.print(duty);
    Serial.print(F(" slider=")); Serial.println(readSlider());
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
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  motorCoast();            // idle free, so the fader can be moved by hand from boot

  for (uint8_t i = 0; i < ARM_COUNT; i++) {
    armAngle[i] = CENTER_ANGLE[i];
  }

  playPattern(PATTERN_HELLO);   // says "I booted" without blocking the loop

  Serial.print(F("OK:READY social-battery ARMS="));
  Serial.println(SERVOS_ENABLED ? ARM_COUNT : 0);
  if (!SERVOS_ENABLED) Serial.println(F("servo arms are PARKED (SERVOS_ENABLED 0) - fader only"));
  Serial.println(F("keys: 1/2/3 = low/med/high, 0 = stop, a = demo cycle, c = hold centre"));
  faderSetup();

  lastTick = millis();
  lastSwitch = 0;               // start the demo cycle immediately
}

void loop() {
  readSerial();
  updateLed();
  faderUpdate();      // the fader runs its own loop, independent of the arms

  unsigned long now = millis();

  // Standalone demo: walk every arm through the states until something takes over.
  if (SERVOS_ENABLED && autoCycle && (lastSwitch == 0 || now - lastSwitch >= DWELL_MS)) {
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
    reportFader();   // its own line, so the S: format stays exactly as it was
  }
}
