# Social Battery

A kinetic object with servo-driven arms on a shared base. It has **three states**, and
each one swings the arm symmetrically about its resting angle — the same number of
degrees left and right of centre — at its own amplitude and speed:

| state | amplitude (each side) | speed | lit dots | reads as |
|---|---|---|---|---|
| Low | 10° | slow | 2 | present, but holding still |
| Medium | 20° | medium | 4 | clearly swaying, comfortable pace |
| High | 35° | slightly faster than medium | 6 | fully switched on — wide and lively |

Where the arms rest does not change between states; only the width and pace of the
sway does.

**Every arm has its own state.** Blue and yellow each get their own Low / Medium / High
buttons on the page, plus an *All arms* row that sets them together (red sits out of both —
see below). Tapping a state starts that arm and sets its speed; tapping another changes
speed and amplitude without the arm jumping. Stop returns it to its resting angle and
**keeps holding it there** — the servo stays powered rather than releasing, since a
released arm goes limp and its own weight droops off `CENTER_ANGLE` under gravity. Detach
one by hand with `E:0` if you need it to move freely, e.g. while re-taping a linkage.

**Red is the exception — it's dial-controlled, not software-controlled.** A potentiometer
wired straight to the board sets red's level directly; see
[Potentiometer — red arm's dial](#potentiometer--red-arms-dial) below. Neither the
standalone demo nor any serial/web command can move it — only the physical dial can.

Built for the **Arduino MKR WiFi 1010**, driven from a web page over USB serial.
The page also runs the same motion model in JavaScript, so the preview animates whether or
not a board is plugged in.

```
SocialBattery/
├── arduino/move-motors/move-motors.ino   the whole sketch — servos, states, serial
└── web/                                  control page (index.html, style.css, app.js)
```

## Wiring — MKR WiFi 1010

All three arms wired, signal on **D5**, **D3** and **D1** — the whole object is live:

| arm | shape | signal pin |
|---|---|---|
| 0 | blue square (longest rod, at the back) | D5 |
| 1 | yellow wedge (middle) | D3 |
| 2 | red octagon (shortest, at the front) | D1 |

```cpp
const uint8_t SERVO_PIN[] = { 5, 3, 1 };
```

Arm order follows that array. Add or change a pin and the sketch adapts on its own — state
handling, telemetry and the arm count reported to the web page all size themselves from it,
and the web preview greys out any arm that has no motor behind it yet.

**Avoid pins 8, 9 and 10.** On the MKR WiFi 1010 those are the SPI bus to the onboard NINA
WiFi module. A servo on one of them appears to work right up until something switches the
radio on.

**D1 is also the board's Serial1 TX pin.** This sketch never uses Serial1, so driving a
servo from D1 is safe here — just keep that in mind if you ever add something that talks
over Serial1 (a GPS module, a second board, etc.), since it would conflict.

The SAMD `Servo` library drives any digital pin from a hardware timer rather than from
`analogWrite`, so the plain digital pins used here — D1, D3 and D5 — all work.

Two things to get right:

- **Power the servo from an external 5V supply, not from the board.** Tie its ground to the
  MKR's ground. The MKR's regulator is not built for motor current, and a servo stalling on
  a 3D-printed linkage will brown out the board mid-movement.
- **The MKR is a 3.3V board**, so the servo receives a 3.3V control pulse while running on
  5V. This is the marginal part of the circuit. Most SG90/MG90S servos accept it, but if the
  arm twitches, stalls, or ignores small movements, that's the cause — a 3.3V→5V level
  shifter on the signal line fixes it. Note also that the MKR's inputs are **not 5V
  tolerant**, which matters the moment you add a sensor to drive the states.

The built-in LED is pin 6 on the MKR (`LED_BUILTIN`), and needs no wiring.

### Potentiometer — red arm's dial

Red (index 2) is driven by a physical dial instead of software: a potentiometer wired to
**A1** sets its level directly, and nothing else can — see `POT_PIN`/`POT_ARM` in the
sketch.

| pot leg | goes to |
|---|---|
| outer leg 1 | **3.3V** |
| outer leg 2 | **GND** |
| middle leg (wiper / "out") | **A1** |

**Never wire either outer leg to 5V.** The MKR's ADC reference is 3.3V; putting 5V across
the divider risks putting more than 3.3V on an analog input pin. The wiper is the signal
pin — it goes to A1, not to a power rail.

Turning the dial reads its position as Off / Low / Medium / High across four even bands,
with a 25-count dead zone at each boundary so it can't flicker between two levels while
sitting near a line. It updates the moment the reading clears into a new band — no
"connect" step, no serial command needed.

This is deliberately exclusive: the standalone demo cycle and every `T`/`X` serial or web
command skip red entirely (`T:2:...` / `X:2` come back `ERR:arm is dial-controlled`), so
turning the knob is the *only* way to set red's level, and nothing else can silently
override it out from under you.

## Run it

1. Open `arduino/move-motors/move-motors.ino` in the Arduino IDE, select
   **Arduino MKR WiFi 1010**, upload. (The IDE requires a sketch's folder and `.ino`
   file to share a name, which is why it lives in `move-motors/move-motors.ino`.)

That is already enough. **On its own the board demonstrates itself**, cycling every arm
through the three states, six seconds each, with no computer attached:

| you should see | |
|---|---|
| LOW | barely moving — a slow drift, but not dead |
| MEDIUM | the full swing, slow and heavy |
| HIGH | the full swing, fast |

To drive it instead:

2. Close the Serial Monitor — it holds the port and the web page will not be able to
   open it. Then serve the web folder (Web Serial needs `localhost` or `https`; opening
   the file directly with `file://` will not work):

```bash
cd "/Users/roshni/Desktop/Claude Projects/SocialBattery/web" && python3 -m http.server 5173
```

3. Open <http://localhost:5173> in **Chrome or Edge** (Safari and Firefox have no Web
   Serial), and click **Connect board**.

The demo cycle stands down the moment a command arrives, so the page and the board never
fight over the arms. Without a board the page falls back to simulation and the preview
animates on its own.

### From the Serial Monitor

Typing the protocol by hand is tedious, so single characters work too — 115200 baud:

| key | |
|---|---|
| `1` / `2` / `3` | all arms to low / medium / high |
| `0` | stop all arms |
| `a` | resume the automatic demo cycle |
| `c` | hold every arm at `CENTER_ANGLE`, for setting the resting pose |

`c` is how you find your resting angles: hold centre, adjust the linkage, and read the
angles off the telemetry line. Digits and lowercase letters were chosen so they cannot
collide with the protocol commands below.

## Connecting, and how the board proves it's there

Opening a serial port is *not* evidence that an Arduino is on the other end. The OS will
happily hand over a Bluetooth modem, or a board sitting there with no sketch on it. So
**Connect board** does a handshake rather than trusting the open port:

1. The page opens the port at 115200 and starts listening.
2. It sends `H` and waits 700ms for an answer, repeating for up to 6 seconds. (The retries
   also cover boards that reset when the port is opened and need a moment to come back.)
3. The board answers `OK:HELLO social-battery ARMS=3` **and blinks the built-in LED**:
   three quick blinks, a beat, then one long blink. Chosen to be unmistakable across a
   room and distinct from the bootloader's own flicker at reset.
4. Only then does the page call itself connected. If nothing answers, it says so, closes
   the port, and stays in simulation rather than pretending.

The LED keeps working as a status light afterwards: **on while any motor is running, off
when all are stopped.**

The page also reads `ARMS=` from the reply, so if you ever wire fewer than three servos it
greys out whichever preview arms have no motor behind them yet.

## Serial protocol

115200 baud, one ASCII command per line, `\n` terminated.

| send | meaning |
|---|---|
| `H` | handshake — blink the LED pattern and identify the board |
| `T:0:HIGH` | start **one** arm in that state |
| `T:MED` | start **every** arm in that state |
| `X:0` | stop one arm |
| `X` | stop every arm |
| `E:0` / `E:1` | detach / attach the servos by hand |
| `?` | ask for one status line now |

| receive | meaning |
|---|---|
| `OK:HELLO social-battery ARMS=1` | answer to `H` |
| `S:HIGH,1,132` | one `state,running,angle` group per arm, `;` separated — 10 Hz |
| `OK:...` / `ERR:...` | command accepted / rejected |

Anything that can open a serial port can drive it — the web page is one client, not the
only possible one.

## Tuning it to the real object

**The resting pose**, one angle per arm — the middle of the swing. Only the first
`ARM_COUNT` entries are used, so the table only needs to grow if you add a fourth arm:

```cpp
const int CENTER_ANGLE[] = { 60, 96, 93 };
```

All three arms are calibrated to the real object — these three angles hold blue, yellow
and red all perfectly vertical. Re-run the same process (hold centre with `c`, nudge and
re-upload until it reads vertical) any time a horn gets re-seated.

`ANGLE_MIN` / `ANGLE_MAX` are a hard clamp applied to every movement — keep them inside
whatever your linkage can physically reach so a stray command cannot strain the mechanism.

**The states.** The whole design lives in two tables:

```cpp
//                                          LOW    MED    HIGH
const float STATE_AMP_DEG[STATE_COUNT]  = { 10.0,  20.0,  35.0 };  // small, medium, large
const float STATE_RATE[STATE_COUNT]     = { 0.35,  0.90,  1.10 };  // slow, medium, slightly faster than medium
```

Amplitude is degrees of swing either side of centre — always symmetrical, so the arm
travels equally far in both directions from `CENTER_ANGLE`; rate is radians per second.
First things to try on the real object: check that low still reads as *alive rather than
off* at 10°, and that high's 35° is inside whatever the linkage can physically clear.
Keep high only a little faster than medium, not a big jump — that's what keeps the three
states reading as a gradient rather than "off, on, faster on."

**Shared phase per state.** Each state (LOW/MED/HIGH) runs its own sine clock, and every
arm currently in that state reads straight off it. Two arms sharing a state are always in
lock step — whichever one joined more recently simply picks up wherever the other already
is, rather than restarting its own cycle. Set an arm to HIGH while another is already
swinging in HIGH and they sway together, the moment the new one catches up on the slew
limiter. Arms in *different* states stay independent, since they're reading different
clocks entirely.

**Other knobs:**

- `STATE_BLEND_SEC` — how long a swing-width change takes to cross over when switching
  states. Only amplitude blends like this; phase jumps straight to the new state's clock,
  which is what keeps arms in sync (the slew limiter still stops that from looking like a
  snap).
- `SLEW_DEG_PER_SEC` — the speed limit on every movement. This is what makes the arm read as
  alive rather than as machinery; lower is heavier and more reluctant.
- `DWELL_MS` — seconds per state in the standalone demo cycle.
- `PATTERN_HELLO` — the connect blink, as on/off millisecond durations starting with ON.

**The same constants are repeated at the top of `web/app.js`.** They are duplicated on
purpose so the page can simulate without hardware — if you retune the sketch, copy the new
values across or the preview will drift away from the object.
