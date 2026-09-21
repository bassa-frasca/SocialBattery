# Social Battery

A kinetic object with two servo arms and a motorized fader on a shared base. Each one
reads out a social-battery level — Off, Low, Medium or High — through how it *moves*,
not where it sits.

```
SocialBattery/
├── arduino/move-motors/move-motors.ino   servos, fader, serial protocol
└── web/                                  control page (index.html, style.css, app.js)
```

> **Naming note.** The firmware's comments still call the fader-driven arm "red,"
> inherited from an earlier version where it really was a red paper octagon on a
> plain rotary potentiometer. The web page now labels that same slot "Yellow" (a
> gourd shape) and calls the second servo "Orange" (a ring) rather than "Yellow" —
> a repaint that happened without the firmware comments catching up. Nothing is
> broken by this, it's just not been reconciled yet; treat "red" in the `.ino` and
> "Yellow" in `app.js`/`index.html` as the same arm.

## The two servo arms

| state | amplitude (of `ARM_REACH`) | speed | reads as |
|---|---|---|---|
| Low | 50% | slow | present, but holding still |
| Medium | 75% | medium | clearly swaying, comfortable pace |
| High | 100% | slightly faster than medium | fully switched on — wide and lively |

Each swings symmetrically about its resting angle in a **ping-pong** motion (a
triangle wave, not a sine) — it crosses the middle at a constant rate and turns
sharply at each end, the same shape the fader's own swing makes, so the two arms and
the fader read as one family of movement rather than two different mechanisms.

**Every servo arm has its own state.** Blue and yellow (orange, per the naming note
above) each get their own Low / Medium / High buttons on the page, plus an *All arms*
row that sets them together. Tapping a state starts that arm and sets its speed;
tapping another changes speed and amplitude without the arm jumping. Stop returns it
to its resting angle.

**Stopped arms release after a settle period** (`RELEASE_AFTER_MS`, 900 ms) rather
than holding torque indefinitely — the opposite of an earlier version of this sketch,
which held forever to guarantee a rigid "vertical at rest." That guarantee cost a
servo stalling continuously (full current, all of it heat) whenever an arm's
`CENTER_ANGLE` sends it into a mechanical stop it can't actually reach — which is
exactly what happened and is why this version releases instead. Set `HOLD_AT_REST`
to `1` in the sketch to go back to permanent holding once `CENTER_ANGLE` is trusted
not to do that.

Built for the **Arduino MKR WiFi 1010**, driven from a web page over USB serial. The
page also runs the same motion model in JavaScript, so the preview animates whether
or not a board is plugged in.

## Wiring — MKR WiFi 1010

| arm | shape | signal pin |
|---|---|---|
| 0 | blue triangle | D2 |
| 1 | orange ring | D3 |

```cpp
const uint8_t SERVO_PIN[] = { 2, 3 };
```

Arm order follows that array. Add or change a pin and the sketch adapts on its own —
state handling, telemetry and the arm count reported to the web page all size
themselves from it.

**Avoid pins 8, 9 and 10.** On the MKR WiFi 1010 those are the SPI bus to the onboard
NINA WiFi module. A servo there works only until something switches the radio on.

Two things to get right with the servos:

- **Power them from an external 5V supply, not from the board.** Tie its ground to
  the MKR's ground. The MKR's regulator is not built for motor current, and a servo
  stalling on a 3D-printed linkage will brown out the board mid-movement.
- **The MKR is a 3.3V board**, so a servo receives a 3.3V control pulse while running
  on 5V. Most SG90/MG90S servos accept it, but if an arm twitches, stalls, or ignores
  small movements, that's the cause — a 3.3V→5V level shifter on the signal line
  fixes it.

The built-in LED is pin 6 on the MKR (`LED_BUILTIN`), and needs no wiring: **on
while any motor is running, off when all are stopped** (plus the connect-handshake
blink pattern).

## The motorized fader (yellow / "red" in the firmware)

A 100mm motorized fader — a slider with its own feedback potentiometer and a DC
motor — driven through an HW-354 H-bridge, with an 8-pixel NeoPixel strip showing
its level. It runs entirely on its own pins and its own loop: it neither drives the
servo arms nor is driven by them, and it isn't part of `ARM_COUNT` at all.

**Today it is hand-only.** Move it and the sketch captures your gesture; let go and
it mirrors that position, ping-ponging between the two at a speed set by the level.
Grab it mid-swing and the motor releases instantly. Software cannot yet *command*
it — see [Open items](#open-items) below.

### Fader wiring

| fader wire | goes to | note |
|---|---|---|
| POT. SLIDER (green) | **A1** | the wiper — position feedback |
| VCC SLIDER (red) | MKR **VCC** (3.3V) | **never 5V** — analog pins are not 5V tolerant |
| GND SLIDER (grey) | MKR **GND** | |
| VCC MOTOR (red) | driver **Motor A / OUT1** | swap with OUT2 to reverse direction |
| GND MOTOR (black) | driver **Motor A / OUT2** | |
| TOUCH SLIDER (orange) | unconnected | capacitive strip, not used yet |

| driver pin | goes to |
|---|---|
| IN1 | **D4** |
| IN2 | **D5** |
| IN3 / IN4 | unused (channel B) |
| VCC / GND | **external 5V supply**, ground tied to MKR GND |

NeoPixel data → **D1**.

**D4 is also `PIN_SPI_SS` on this board's variant** — a known-risky alias (a servo or
motor pin fighting the SPI alias is a subtle failure), documented here rather than
avoided, since it's what's actually wired. If the fader ever misbehaves in a way
power/wiring doesn't explain, suspect this pin first — moving IN1 elsewhere (D7 was
the previous home) is the way to test whether the symptom follows it.

**Motor power must not come from the MKR.** On the 3.3V pin the fader crawls at
~13 counts/sec and repeatedly browns the board off USB; on an external 5V supply it
manages a full sweep in well under a second. Grounds tied, and the fader's own
position-feedback pot still runs on 3.3V regardless of the motor's supply.

### How it reads a level

The fader's *level* is how far the slider sits from the **centre** of its travel, in
either direction — so the scale is symmetric, and a position and its mirror are
always the same level:

```cpp
#define SLIDER_MIN 336      // reading at the low mechanical stop
#define SLIDER_MAX 753      // reading at the high stop
```
(measured on the bench — not the full 0–1023 the ADC can report)

Half of that travel is the furthest the slider can sit from centre, and that
distance divides evenly into four bands — Off, Low, Medium, High — each the same
width either side of the middle, with an 8-count dead zone at each boundary
(`BAND_HYSTERESIS`) so noise on a mark can't flicker the level.

The NeoPixel strip shows the level you **set**, not wherever the slider happens to
be mid-swing (otherwise it would flicker through every colour on every pass):

| level | pixels | colour |
|---|---|---|
| Off | none | — |
| Low | the middle pair | `#FFC400` yellow |
| Medium | four, widening | `#FF6A00` orange |
| High | all eight | `#FF1FA0` magenta |

How hard each level drives the return swing:

```cpp
const int SWING_SPEED[4] = { 0, 230, 243, 255 };   // Off, Low, Medium, High
```

### Hand detection

While the motor drives toward a target, the slider should get steadily closer to it.
If it instead moves the *other* way by more than a small margin, that's a hand —
motor released immediately, no distance threshold to clear first, so a grab is caught
from the very start of a leg, not only once it's pushed back some distance.

A gesture also has to be big enough to count (`MIN_GESTURE`) before it re-picks the
level — otherwise a stall near one end of a swing looks exactly like a hand letting go
right there, and would silently promote the level to whatever that end happens to be.

If the slider stalls mid-drive (friction, not a hand), the sketch winds the duty up
step by step until it moves again, up to a ceiling — past that ceiling it assumes a
hand is holding it still rather than pushing harder into what would otherwise be a
stall.

## Open items

- **`FD:` telemetry** exists and is sent (`FD:<pos>,<band>,<seeking>`, 10 Hz,
  alongside `S:`) — this was the main gap between this sketch and the web page that
  was already written to expect it, and it's now closed.
- **Software cannot drive the fader yet.** The protocol comment used to claim
  `T:<arm>`/`X:<arm>`/`F:<pos>` could command it — none of that was actually wired up
  (no case for any of them in `handleLine()`). Setting red/yellow's level from the
  web page or over serial, with the physical fader moving to match, is real future
  work: a target-seeking mode for `faderUpdate()` that the existing hand-detection
  logic can still interrupt.
- **Fader calibration isn't wired to a key.** `calibrate()` (drive to each end,
  measure travel, print `SLIDER_MIN`/`SLIDER_MAX` to paste back in) exists but isn't
  called from anywhere — an earlier version of this file's header comment promised a
  `k` key for it that was never implemented. `SLIDER_MIN`/`MAX` above are the last
  bench measurement, hand-pasted in.
- **Arm/colour naming** — see the note at the top.

## Run it

1. Open `arduino/move-motors/move-motors.ino` in the Arduino IDE, select
   **Arduino MKR WiFi 1010**, upload. (The IDE requires a sketch's folder and `.ino`
   file to share a name, which is why it lives in `move-motors/move-motors.ino`.)

On its own the board demonstrates the two servo arms, cycling them through the three
states, six seconds each, with no computer attached — the fader, having no
standalone demo, just sits waiting for a hand the whole time.

To drive the servo arms from the page instead:

2. Close the Serial Monitor — it holds the port and the web page will not be able to
   open it. Then serve the web folder (Web Serial needs `localhost` or `https`;
   opening the file directly with `file://` will not work):

```bash
cd "web" && python3 -m http.server 5173
```

3. Open <http://localhost:5173> in **Chrome or Edge** (Safari and Firefox have no Web
   Serial), and click **Connect board**.

The demo cycle stands down the moment a command arrives. Without a board the page
falls back to simulation and the preview animates on its own.

### From the Serial Monitor

Typing the protocol by hand is tedious, so single characters work too — 115200 baud,
and these only ever touch the two servo arms:

| key | |
|---|---|
| `1` / `2` / `3` | both servo arms to low / medium / high |
| `0` | stop both servo arms |
| `a` | resume the automatic demo cycle |
| `c` | hold both servo arms at `CENTER_ANGLE`, for setting the resting pose |

`c` is how you find your resting angles: hold centre, adjust the linkage, and read
the angles off the telemetry line.

## Connecting, and how the board proves it's there

Opening a serial port is *not* evidence that an Arduino is on the other end. So
**Connect board** does a handshake rather than trusting the open port: it sends `H`
and waits for `OK:HELLO social-battery ARMS=<n>` plus a blink pattern (three quick
blinks, a beat, one long blink) before calling itself connected. If nothing answers,
it says so and stays in simulation.

## Serial protocol

115200 baud, one ASCII command per line, `\n` terminated. Everything below addresses
the two **servo** arms (0=blue, 1=orange) only — the fader has no commands yet, see
[Open items](#open-items):

| send | meaning |
|---|---|
| `H` | handshake — blink the LED pattern and identify the board |
| `T:0:HIGH` | start **one** servo arm in that state |
| `T:MED` | start **both** servo arms in that state |
| `X:0` | stop one servo arm |
| `X` | stop both servo arms |
| `V` | print the live amplitude/speed table |
| `V:MED:24:0.7` | retune a state's amplitude and speed live, no re-upload |
| `E:0` / `E:1` | detach / attach the servos by hand |
| `?` | ask for one status line now (both `S:` and `FD:`) |

| receive | meaning |
|---|---|
| `OK:HELLO social-battery ARMS=2` | answer to `H` |
| `S:HIGH,1,132;MED,0,96` | one `state,running,angle` group per servo arm, `;` separated — 10 Hz |
| `FD:544,0,0` | fader telemetry — `position,level(0-3),seeking(0/1)` — 10 Hz |
| `OK:...` / `ERR:...` | command accepted / rejected |

Anything that can open a serial port can drive it — the web page is one client, not
the only possible one.

## Tuning it to the real object

**The resting pose**, one angle per servo arm — the middle of the swing:

```cpp
const int CENTER_ANGLE[] = { 96, 96, 93 };
```

Only the first `ARM_COUNT` (2) entries are used. Arm 1 (orange) is confirmed
vertical at 96 on the real object. Arm 0 (blue) is a safe starting value, not yet
measured — it was previously calibrated to 60, which put HIGH's sweep low enough to
stall the linkage against its mechanical stop and cook the servo. Use `c` to hold
centre, adjust, and read the true vertical off the telemetry line before trusting a
new value. The third entry is inert (`ARM_COUNT` is 2), left over from when this arm
existed.

`ANGLE_MIN` / `ANGLE_MAX` are a hard clamp applied to every movement — keep them
inside whatever your linkage can physically reach.

**The states.** Amplitude is a fraction of `ARM_REACH` (how far an arm can swing
either side of centre before clearing every level); rate is radians per second,
live-tunable with `V:` without a re-upload:

```cpp
#define ARM_REACH 40.0
float stateAmp[STATE_COUNT]  = { ARM_REACH * 0.50, ARM_REACH * 0.75, ARM_REACH };
float stateRate[STATE_COUNT] = { 0.35, 0.90, 1.10 };
```

**Shared phase per state.** Each state (LOW/MED/HIGH) runs its own clock, and every
arm currently in that state reads straight off it — two arms sharing a state are
always in lock step, whichever joined more recently simply picking up wherever the
other already is. Arms in *different* states stay independent.

**Other knobs:**

- `STATE_BLEND_SEC` — how long a swing-width change takes to cross over when
  switching states. Only amplitude blends like this; phase jumps straight to the new
  state's clock, which is what keeps arms in sync (the slew limiter still stops that
  from looking like a snap).
- `SLEW_DEG_PER_SEC` — the speed limit on every movement. Lower is heavier and more
  reluctant.
- `DWELL_MS` — seconds per state in the standalone demo cycle.
- `RELEASE_AFTER_MS` / `HOLD_AT_REST` — see [The two servo arms](#the-two-servo-arms)
  above.

**The same constants are duplicated at the top of `web/app.js`** so the page can
simulate without hardware — if you retune the sketch, copy the new values across or
the preview will drift away from the object.
