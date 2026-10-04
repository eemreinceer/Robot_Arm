# ESP32 synchronized smooth-motion servo controller

PlatformIO firmware for an **ESP32 DevKit V1 (WROOM-32)** driving the Robot Arm
arm's six 50 Hz RC-servo PWM signals:

- three MG996R servos (`joint_1..joint_3`);
- three SG90 servos (`joint_4`, `joint_5`, `joint_6/gripper`).

The firmware provides two deliberately separate command paths:

1. **Legacy UART protocol v1**, byte-for-byte compatible with the deployed
   `arm_hardware` host (`V?`, `P...`, `S`).
2. **Additive smooth-motion protocol**, with operator-reference arming,
   quintic synchronized trajectories, speed limiting, replace/queue modes,
   STOP/ESTOP, status and communication timeout.

No firmware from this directory is uploaded automatically. A successful build
or native test is not physical acceptance.

## Hardware and transport

| Function | ESP32 GPIO | LEDC channel | Servo |
| --- | ---: | ---: | --- |
| `joint_1` base | 13 | 0 | MG996R |
| `joint_2` shoulder | 14 | 1 | MG996R |
| `joint_3` elbow | 25 | 2 | MG996R |
| `joint_4` | 26 | 3 | SG90 |
| `joint_5` | 27 | 4 | SG90 |
| `joint_6` gripper | 33 | 5 | SG90 |
| heartbeat LED | 2 | — | on-board LED |

The protocol runs over **UART0**, the same hardware UART the onboard CP2102
exposes through the USB port. Connect the ESP32 to the host with a USB cable;
no GPIO wiring to the host is needed. Enumerates as `/dev/ttyUSB0`.

Settings: `/dev/ttyUSB0`, **115200 8N1**.

⚠ Until 2026-08-19 this ran over UART2 (GPIO16/17), wired directly to the
Jetson Nano's 40-pin header. That path no longer exists: the firmware only
listens on UART0 now. GPIO16/17 are free for other use.

Servo power is external. Servo VCC never comes from ESP32 5 V/3.3 V. Servo
supply, ESP32 and the host require a common signal-reference GND, but servo
return current must not flow through the ESP32 board.

## Architecture

```text
UART2 bytes
  └─ main.cpp fixed line buffer
      ├─ legacy protocol.c       -> direct calibrated pulse path
      └─ smooth_protocol.cpp     -> validated typed command
          └─ motion_controller.cpp
              ├─ state machine and fixed FIFO
              ├─ synchronized quintic profile
              ├─ deadband and peak-speed duration guard
              └─ angleToPulseUs()
                  └─ main.cpp LEDC hardware adapter
```

Files:

- `include/robot_config.hpp`: calibration type plus motion constants.
- `include/robot_config_generated.hpp`: checked-in calibration generated from
  the canonical ROS YAML; never edit it manually.
- `tools/generate_robot_config.py`: validates the YAML and regenerates/checks
  the firmware calibration.
- `include/motion_controller.hpp`, `src/motion_controller.cpp`: platform-free
  trajectory/state/queue code.
- `include/smooth_protocol.hpp`, `src/smooth_protocol.cpp`: fixed-buffer CSV
  parser with strict field and number validation.
- `src/main.cpp`: Arduino UART2, LEDC and 20 ms scheduler adapter.
- `test/`: native host tests; no connected servo is needed.
- `protocol.c` / `protocol.h`: symlinks to the locked STM32 protocol v1
  implementation. They are not duplicated or modified here.

There is no `delay()` and no Arduino `String` in the control path. UART parsing,
the safety checks, LED heartbeat and servo updates can all make progress in the
same loop.

## Servo configuration

The canonical calibration source is
`src/robot_arm_description/config/servo_calibration.yaml`.
`ServoCalibration` keeps these concepts separate:

- calibrated servo-angle anchors corresponding to the pulse endpoints;
- narrower mechanical/safety limits expressed in the ROS joint frame;
- the measured ROS-joint-to-servo zero offset;
- maximum normal and startup speeds;
- deadband and installation reversal.

This separation is critical: narrowing a safety limit must not silently
rescale angle-to-pulse calibration.

Regenerate and verify after every calibration change:

```bash
cd firmware/esp32_servo_ctrl
python3 tools/generate_robot_config.py
python3 tools/generate_robot_config.py --check
```

The generated header includes a semantic SHA-256 calibration identifier.
`PING` and `STATUS` expose its first 12 characters so a host can reject a
mismatched firmware image. GPIO/LEDC wiring, startup-speed policy and deadband
remain firmware-local; pulse anchors, limits, zero offsets, polarity and
normal speed come from the YAML.

The firmware mapping is deliberately the same as `arm_hardware`:

```text
servo_angle = ros_joint_angle + zero_offset
ratio = (servo_angle - calibration_min) / (calibration_max - calibration_min)
pulse = min_us + ratio * (max_us - min_us)
```

The ROS joint target is clamped before this mapping. Installation reversal is
applied to the ratio. Native parity tests compare the host-reference and actual
firmware functions at five points per joint with a tolerance of `1 µs`.

## PWM and scheduler

LEDC runs at 50 Hz and 16-bit resolution. One count is approximately
`0.305 µs`. Motion is evaluated every 20 ms. Smooth-motion angles map
directly from floating-point pulse widths to LEDC counts, so the available
sub-microsecond resolution is preserved. The locked legacy `P` path keeps its
deployed integer-microsecond conversion unchanged.

If the loop is late, it performs one current-time update instead of emitting an
unbounded burst of stale samples. All elapsed-time checks use unsigned
subtraction and remain valid across `millis()` wraparound. The main loop
samples time after RX handlers, and the elapsed-time helper rejects a
just-recorded timestamp that is one tick newer than the loop sample; this
prevents a false immediate watchdog/communication timeout.

Every synchronized move snapshots all joint estimates at one instant. Every
joint uses the same normalized time and therefore starts and finishes together:

```text
s(t) = 6t^5 - 15t^4 + 10t^3
q(t) = q_start + (q_target - q_start) * s(t)
```

The final update writes the exact target. The smootherstep peak slope is
`1.875`, so duration limiting uses:

```text
minimum_duration =
  1.875 * abs(target_deg - start_deg) / max_speed_deg_per_sec
```

This is more conservative than limiting only average speed. If a requested
duration is too short, it is extended and the accepted duration is returned in
the ACK.

## Startup state machine

Default startup policy is **WAIT_FOR_ARMED_COMMAND**:

1. Boot attaches LEDC channels with duty zero; no servo pulse is emitted.
2. State is `DISARMED`.
3. `ARM` requires six operator-observed ROS joint angles in degrees.
4. `ARM` records the estimate but still emits no PWM.
5. The first accepted `MOVE`, `JOINT`, `NEUTRAL` or `DEMO` writes the supplied
   reference pulse, then follows its limited profile.

The ESP32 cannot measure actual RC-servo position. `ARM` values are an operator
estimate, not encoder feedback. A wrong reference can still cause a jump when
PWM first appears. Support the arm, keep the physical servo-rail cutoff in
hand and commission one joint at a time.

State transitions:

```text
BOOTING -> DISARMED --ARM,<6 references>--> READY
READY --DISARM/timeout--> DISARMED
READY --ESTOP--> ESTOP
ESTOP --RESET_ESTOP--> DISARMED
```

After `RESET_ESTOP`, a fresh `ARM` is mandatory.

## Legacy protocol v1

The deployed host contract remains unchanged:

| Send | Reply | Behaviour |
| --- | --- | --- |
| `V?` | `V1,1.2.1-esp32` | version handshake |
| `P500,1000,1500,2000,2500,1750` | `OK` | direct pulse update |
| `S` | `OK` | all PWM off |
| malformed legacy line | `E1` | rejected |
| wrong P field count | `E2` | rejected |
| 1 s without another P | `E3` once | last PWM remains active |

Legacy P exists only for compatibility with `arm_hardware`, which already
performs host-side activation, rate limiting and calibration. It intentionally
does not run a second long S-curve inside the ESP32. The new smooth protocol is
the firmware-owned trajectory path.

If smooth-protocol ESTOP is latched, legacy P is rejected with
`ERR,ESTOP_LATCHED`. Legacy `S` may cut PWM but does not clear the latch.
If legacy PWM is active, the smooth path cannot take ownership until `S`
turns those outputs off; this prevents an unmeasured legacy pulse estimate
from being replaced by a guessed angle.

## Smooth-motion protocol

Every packet is ASCII CSV terminated by `\n`. Maximum line length is 192 bytes.
Empty fields, extra fields, malformed integers/floats, `NaN`, infinity,
overflow, unknown commands and unknown modes are rejected.

All smooth-protocol angles are **ROS joint positions in degrees**: the same
joint coordinate used by URDF/ros2_control, converted from radians to degrees.
They are not raw servo-shaft angles. The firmware applies the generated
`zero_offset_rad` internally. Joint numbers are one-based.

This coordinate contract applies to `ARM`, `MOVE`, `JOINT`, `NEUTRAL`, `DEMO`
and the angles returned by `STATUS`. `NEUTRAL` means ROS `q=[0,0,0,0,0,0]`.

### Commands

```text
PING
STATUS
ARM,q1,q2,q3,q4,q5,q6
DISARM
MOVE,q1,q2,q3,q4,q5,q6,duration_ms,REPLACE[,motion_id]
MOVE,q1,q2,q3,q4,q5,q6,duration_ms,QUEUE[,motion_id]
JOINT,joint,angle_deg,duration_ms,REPLACE[,motion_id]
JOINT,joint,angle_deg,duration_ms,QUEUE[,motion_id]
NEUTRAL,duration_ms[,REPLACE|QUEUE][,motion_id]
STOP
ESTOP
RESET_ESTOP
CLEAR_QUEUE
DEMO
```

Examples:

```text
ARM,0,0,0,0,0,0
MOVE,10,-8,5,4,-3,0,2500,REPLACE,42
JOINT,2,-12,1500,QUEUE,43
NEUTRAL,3000,QUEUE,44
PING
STATUS
```

Typical replies:

```text
ACK,ARM,PWM=OFF
ACK,MOVE,DURATION_MS=3200,ID=42
ACK,JOINT,NOOP=DEADBAND,ID=43
ERR,ANGLE_LIMIT
ERR,INVALID_NUMBER
ERR,QUEUE_FULL
STATE,READY,...,FRAME=ROS_JOINT_DEG,CALIB=<12 hex>
ANGLES,CURRENT=...,...,TARGET=...,...
EVENT,MOTION_COMPLETE,ID=42
```

### Replace and queue

- `REPLACE`: evaluate the active trajectory at the receipt timestamp, snapshot
  that interpolated estimate, clear the queue and start from that position. It
  never jumps back to the previous trajectory start.
- `QUEUE`: append an atomic whole-pose command to a fixed four-entry FIFO. A
  single-joint queued command inherits the latest planned pose for the other
  joints. Full queues return `ERR,QUEUE_FULL`.
- `CLEAR_QUEUE`: removes waiting entries without stopping the active move.

Targets inside every affected servo's deadband are acknowledged as a no-op and
do not restart the trajectory.

### STOP, ESTOP, DISARM and S

- `STOP`: evaluates the trajectory now, holds that estimated position and
  clears the queue. State remains `READY`.
- `DISARM`: stops motion, clears the queue, holds the last PWM and requires a
  fresh `ARM`.
- `ESTOP`: immediately cancels motion, clears the queue, holds the last PWM and
  latches `ESTOP`. Holding is the default because cutting PWM can make a
  gravity-loaded arm fall.
- `RESET_ESTOP`: returns to `DISARMED`; it never resumes old motion.
- legacy `S`: physically turns PWM off. It may release a loaded arm.

Software ESTOP is not a physical emergency stop. The real servo-power cutoff
must remain reachable.

### Communication timeout

While the smooth path is `READY`, a valid smooth command or `PING` must arrive
at least every 2000 ms. Timeout:

1. samples the current interpolated estimate;
2. stops the active trajectory;
3. clears the queue;
4. holds PWM;
5. enters `DISARMED`;
6. emits `EVENT,COMM_TIMEOUT,STATE=DISARMED,PWM=HOLD`.

Recovery requires a fresh physical reference and `ARM`. Timeout is intentionally
different from latched ESTOP.

### Safe demo

`DEMO` is disabled by state, not by a hidden compile-time auto-start. It works
only after an explicit reference `ARM` and queues:

1. small offsets around the supplied reference;
2. opposite small offsets;
3. ROS joint zero.

It uses startup speed limits, fixed non-blocking motions and never approaches a
configured limit deliberately. It still moves real hardware.

## Servo-rail monitor (Faz 7 / A1)

A measurement-only path. It never commands motion, never touches PWM and is
not part of the smooth-motion safety contract.

There is no oscilloscope on this bench. The ESP32 ADC is the fastest
instrument available: sampled continuously at 20 kHz it resolves tens of
microseconds, where an INA226 conversion takes at least 140 µs.

### Wiring

```text
servo connector V+ ──[ 20 kΩ ]──┬── GPIO34
                                │
                            [ 10 kΩ ]
                                │
servo connector GND ────────────┴── ESP32 GND (common)
```

**Probe the servo connector, not the buck output.** The required measurement is
the voltage at the servo's own connector: the buck can read 5.5 V while the
servo sees less because of wiring or connector loss.

GPIO34 is ADC1 (ADC2 stops converting when WiFi is up) and input-only. The
divider draws ~200 µA, so it cannot cause the sag it measures. **No filter
capacitor on the node** — the transient is the signal.

Constants live in `include/rail_config.hpp`. Change them there if the divider
changes; the scale is nominal and a multimeter anchors it once at a steady
state. What this measures is *relative* collapse.

### Commands

| Send | Reply |
| --- | --- |
| `RAIL` | `RAIL,MV=,MIN=,MAX=,MEAN=,MIN1S=,N=,TRIG=,PIN=,SAMPLER=` |
| `RAILRESET` | `ACK,RAILRESET` — clears statistics and any capture |
| `RAILTRIG,<mv>` | `ACK,RAILTRIG,MV=` — arm single-shot capture below `<mv>` |
| `RAILTRIG,OFF` | `ACK,RAILTRIG,OFF` |
| `RAILDUMP` | `RAILDUMP,BEGIN,...` then `RD,<i>,<rel_us>,<raw>,<mv>` lines, then `RAILDUMP,END` |

`MIN1S` is the lowest rail seen in the rolling ~1 s window; `MIN` is the lowest
since the last reset. A capture keeps ~102 ms either side of the event at
20 kHz.

These commands **deliberately do not refresh the smooth-motion communication
timeout**. A host that only polls the rail is not proving the control link is
alive and must not be able to hold the arm armed.

`RAILDUMP` requires a completed capture and refuses while a trajectory is
running: the transfer takes seconds at 115200 baud and the frozen capture will
keep. Freezing is also what lets the dump run without blocking the sampler.

### Session shape

```text
RAILRESET
RAILTRIG,5000          # arm below 5.0 V
<move one joint, then two, then more>
RAIL                   # MIN/MIN1S say whether the rail sagged at all
RAILDUMP               # only if the trigger fired; plot rel_us vs mv
```

Core 0 is dedicated to the sampler and its idle watchdog is disabled, so this
firmware must not bring up WiFi or Bluetooth. If either is ever needed, move
the sampler to a timer-driven design first.

## Build, test and upload

From this directory:

```bash
pio run -e esp32dev
pio test -e native
python3 tools/generate_robot_config.py --check
```

The native tests cover:

- smootherstep endpoints and monotonicity;
- ROS-joint clamp and angle/pulse mapping, including zero offsets and reversed
  installation;
- generated-calibration freshness and host/firmware PWM parity within `1 µs`;
- peak-speed duration extension and slowest-joint synchronization;
- deadband, exact final target and replace continuity;
- fixed queue capacity and transition to the next move;
- ESTOP latch/reset/re-arm;
- valid/invalid UART commands, field counts, modes, `NaN` and line length.

Upload is a separate, explicit physical operation:

```bash
pio run -e esp32dev -t upload
```

After upload, verify without servo power:

```text
V? -> V1,1.2.1-esp32
PING -> ACK,PING,FW=1.2.1-esp32,FRAME=ROS_JOINT_DEG,CALIB=<12 hex>
STATUS -> STATE,DISARMED,...,PWM=0,...
```

## Manual calibration and commissioning

Do not discover pulse endpoints automatically.

1. Support the arm against gravity and switch servo power off.
2. Verify joint-to-GPIO mapping and polarity.
3. Disconnect all but the joint being commissioned if practical.
4. Verify firmware/version and `DISARMED`, `PWM=0` with servo power still off.
5. Place the joint at a measured, reachable physical angle.
6. Send `ARM` with all six observed ROS joint angles in degrees. Confirm it
   replies `PWM=OFF`.
7. Turn on servo power with the physical cutoff in hand.
8. Send a small, slow single-joint move well inside the known limits.
9. Confirm physical direction and actual settling visually; firmware angles are
   estimates, not measurements.
10. Repeat toward limits in small steps. Stop at mechanical load/noise/heat;
    never command `500/2500 µs` blindly on an assembled mechanism.
11. Update the host `servo_calibration.yaml`, regenerate
    `robot_config_generated.hpp`, and record the calibration ID.
12. Re-run the freshness/parity checks, native tests, build, pin-level
    logic-analyzer checks and the same
    single-joint commissioning before multi-joint motion.

The six 470 µF rail capacitors may improve local supply transients, but they do
not validate grounding, connector current, servo calibration or motion safety.

## Physical validation status

The earlier v1 firmware, UART2 link and selected PWM pins were physically
measured. This **smooth-motion firmware version has only been built and tested
on the host unless a later field report says otherwise**. Required acceptance
after an authorized flash:

1. boot with PWM low and `STATE=DISARMED`;
2. six pin-level 50 Hz pulse captures;
3. one unloaded servo, small move, STOP and timeout;
4. one mounted joint at startup-speed limits;
5. six-joint synchronized small move with current/rail/video evidence;
6. queue, replace, communication loss and ESTOP recovery.
