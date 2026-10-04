# arm_hardware

ROS 2 control `SystemInterface` for the Robot Arm's six-channel STM32 servo controller.

The plugin exports a position command plus position/velocity state for `joint_1` through
`joint_6`. RC servos are open-loop, so `read()` reports the position represented by the
last PWM command actually delivered after calibration clamp and slew limiting, with zero
velocity. This is still an estimate, not encoder feedback. `write()` maps radians through
`servo_calibration.yaml`, applies the
configured pulse-width slew limit, and sends the locked UART v1 command:

```text
P1500,1500,1500,1500,1500,1500\n
```

The YAML `channel` field is authoritative: ROS joint order may differ from physical
STM32 channel order. Duplicate physical channel assignments are rejected at startup.

Activation performs a `V?` / `V1,<firmware>` handshake but starts **disarmed**. While disarmed,
`write()` emits no `P` frame. No upload or firmware flashing is performed by this package.

## Activation safety

The plugin owns a small parameter node named `/robot_arm_hardware_safety`. Arming is a deliberate
two-step operation because these servos have no encoder feedback:

```bash
ros2 param set /robot_arm_hardware_safety reference_positions \
  "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
ros2 param set /robot_arm_hardware_safety armed true
```

The six reference values are the operator's estimate of the arm's current physical pose in ROS
radians, ordered `joint_1` through `joint_6`. Arming without a valid reference is rejected. The
arming transition sends no PWM frame: the next control target starts from the reference and is
limited by both `max_velocity_rad_s` and `soft_start.max_delta_us_per_step`.

Disarm at any time with:

```bash
ros2 param set /robot_arm_hardware_safety armed false
```

An armed-to-disarmed transition emits exactly one `S` frame and invalidates the old reference.
Rearming therefore requires a newly observed physical reference pose. This ROS command is not an
emergency stop; the physical servo-rail disconnect remains mandatory during commissioning.

Each calibration channel accepts an optional `max_velocity_rad_s` (safe default `0.1`). The YAML
root also accepts:

```yaml
activation_safety:
  calibration_complete: false
  commissioning_range_rad: 0.3
```

Until `calibration_complete` is explicitly true, every target must remain within the calibrated
joint bounds and within `reference_positions ± commissioning_range_rad`. Out-of-range and non-finite
commands are rejected rather than silently coerced. Setting `calibration_complete: true` is a field
acceptance decision; it must not be used merely to bypass the commissioning gate.

## Hardware parameters

- `serial_device` (default `/dev/ttyUSB0`)
- `baud_rate` (default `115200`, 8N1)
- `mock_serial` (`true` skips the physical port and provides a deterministic loopback)
- `command_rate_hz` (default `50`, maximum `50`)
- `serial_response_timeout_ms` (default `20`, must be `> 0` and `<=` the command period)
- `calibration_file` (required)

### `serial_response_timeout_ms` — why it is its own parameter

Until 2026-07-28 the ACK window was the soft-start `step_period_ms` from the calibration YAML.
Those are different physical quantities: `step_period_ms` governs how fast PWM is allowed to ramp,
while the ACK window governs how long the complete host-TX to MCU-ACK transaction may take. Tuning
motion smoothness silently retimed the serial link.

The default is deliberately `20` ms — identical to the old effective value — so the split alone
changes no field behaviour. Choose the real value from the `ACK exchange latency` lines the
interface logs (periodically while running, and as a summary on deactivate); raise it if `max`
approaches the window. TX and ACK reception share one absolute deadline; the driver does not grant
a fresh full timeout to each phase. Because `write()` waits for the ACK inside the real-time loop,
the total window cannot exceed the command period, and `on_init` refuses a configuration where it
does. Latency is measured from the start of host TX until the complete ACK is consumed.

Observed failure signature when the window is too tight:

```
ESP32 ACK timed out after 20 ms with a partial line 'O' (1 bytes)
```

That is a split `OK\n`, not a rejected command. A failed exchange drains any late tail before
failing closed, so the next exchange cannot mistake the previous reply for its own ACK.

The timeout limits **waiting**, not processing data the UART driver has already buffered. If the
control thread is descheduled after reading `O` and resumes after the nominal deadline, the reader
performs one zero-wait drain of immediately readable bytes. A complete buffered `OK\n` is accepted
and counted as a `buffered deadline completion`; no grace sleep is added. If the tail is not already
available, the exchange still times out, drains late bytes, and enters the same fail-closed error
transition. This keeps the 20 ms communication budget while avoiding a false timeout caused only by
host scheduling between bytes of one three-byte ACK.

On teardown the driver emits a fixed-cost latency distribution for successful
command-to-ACK exchanges. The 50 Hz path only increments one preallocated
counter: 1 ms buckets through the 20 ms control deadline, followed by 25 ms,
50 ms and overflow buckets. Formatting and percentile calculation happen only
after the interface is made inactive and the serial device is closed. The log
contains nearest-rank p50/p95/p99 bucket bounds plus every bucket count; retain
that line with the exchange/timeout/buffered-completion summary for the 100k
acceptance artifact. Timeout exchanges remain a separate counter and are not
silently mixed into the successful-latency histogram. All counters reset after
each successful activation handshake, so separate acceptance runs cannot be
combined accidentally.

### Nano scheduling preflight (read-only)

Do not start a stopped container merely to run these checks. The first command
can inspect the stored `robot_arm_hw` definition without starting it:

```bash
docker inspect robot_arm_hw --format \
  'running={{.State.Running}} pid={{.State.Pid}} user={{json .Config.User}} cap_add={{json .HostConfig.CapAdd}} ulimits={{json .HostConfig.Ulimits}}'
```

If an operator has already authorized a ray-OFF stack and `robot_arm_hw` is running,
capture the effective limits and every thread's scheduler class. These commands
only read process state; they do not arm hardware, claim UART, or change
priority:

```bash
docker exec robot_arm_hw sh -lc \
  'printf "rtprio="; ulimit -r; printf "memlock_kb="; ulimit -l; id; uname -a'
docker exec robot_arm_hw sh -lc \
  'ps -eLo pid,tid,cls,rtprio,pri,psr,comm,args | grep -E "PID|ros2_control|controller_manager"'
```

Acceptance requires the actual controller thread, not just container metadata,
to show the intended real-time class/priority without the Controller Manager
`Could not enable FIFO RT scheduling policy` warning. Preserve `docker inspect`,
the two command outputs, the complete controller startup log and the ACK
distribution together. Missing permission is a deployment finding; do not use
`sudo`, change limits, restart the container, or open the servo rail during this
read-only preflight.

### ESP32 legacy ACK static audit (2026-08-14)

The deployed legacy path was reviewed without flashing or running hardware.
A representative 31-byte `P...\n` frame occupies about `2.691 ms` on 115200
8N1; its three-byte `OK\n` reply adds about `0.260 ms`. Firmware parsing is a
fixed six-field `strtol` loop. A valid position command disarms the unused
smooth-motion state, performs exactly six `ledcWrite` calls and then writes
`OK\n`; there is no `delay()`, dynamic `String`, motion queue wait or unbounded
numeric loop on that path.

The audited PlatformIO environment resolves Arduino-ESP32
`3.20017.241212+sha.dcc1105b`. Its `HardwareSerial::write` takes the UART mutex
with an unbounded RTOS wait and calls `uart_write_bytes`; the default TX buffer
is zero, so that API may wait until the short reply is sent or placed in the
hardware FIFO. This firmware has no competing UART2 writer task, making mutex
contention unlikely in the current code, but a static review is not a measured
worst-case bound. `processRx()` also drains all currently queued input rather
than enforcing a per-loop byte budget; that is a starvation risk under a
flood/batched-command fault, not under the host's normal one-command/one-ACK
exchange.

No firmware change is justified by this audit alone. The remaining timing
unknown must be separated with host scheduling evidence and, if needed, a
passive UART capture: host TX start, complete command on the wire, first ACK
byte and ACK newline. Do not add a timestamp or sequence field to legacy `OK\n`
without an approved protocol revision because the deployed host intentionally
requires the exact response.

For a hardware-free lifecycle smoke test, use `robot_arm_real.urdf.xacro` with
`mock_serial:=true`. The production xacro defaults to `false` deliberately.
