# ESP32 Smooth Motion Controller Report

**Date:** 2026-07-24; Stage 1 calibration-contract update 2026-07-25

**Scope:** `firmware/esp32_servo_ctrl/`

**Physical flash/motion:** not performed

## İncelenen mevcut yapı

- **Kullanılan framework:** PlatformIO, `espressif32`, Arduino framework,
  ESP32 DevKit V1/WROOM-32.
- **Servo kütüphanesi:** external Servo library yok; ESP32 LEDC hardware PWM,
  50 Hz / 16 bit.
- **Servo pinleri:** GPIO `13,14,25,26,27,33`; LEDC channels `0..5`.
- **UART ayarları:** UART2, GPIO16 RX / GPIO17 TX, 115200 8N1,
  Jetson `/dev/ttyTHS1`.
- **Mevcut protokol:** locked v1 `V?`, `P<pulse_us x6>`, `S`,
  `OK/E1/E2/E3`.
- **Başlangıç davranışı:** boot PWM off; ilk valid P doğrudan pulse yazıyordu.
- **Build/test:** ESP32 `pio run`; shared legacy parser
  `firmware/stm32_servo_ctrl` native testleri.
- **Mevcut önemli sorunlar:** firmware-owned motion profile/state/queue yoktu;
  P hedefleri doğrudan uygulanıyordu; parser/pulse/hardware tek dosyadaydı;
  ESP32 smooth-motion host testleri yoktu.

## Yapılan değişiklikler

### Oluşturulan dosyalar

- `include/robot_config.hpp`
- `include/robot_config_generated.hpp`
- `include/motion_controller.hpp`
- `include/smooth_protocol.hpp`
- `src/motion_controller.cpp`
- `src/smooth_protocol.cpp`
- `test/test_motion_controller/test_main.cpp`
- `test/test_smooth_protocol/test_main.cpp`
- `tools/generate_robot_config.py`
- `tools/platformio_check_calibration.py`

### Değiştirilen dosyalar

- `src/main.cpp`
- `platformio.ini`
- `README.md`

### Eklenen ana özellikler

- Servo pin/pulse/angle/limit/speed/deadband configuration tek yapıda.
- Pulse calibration anchors ile safety limits ayrıldı; limit daraltması
  angle-to-pulse ölçeğini değiştirmiyor.
- 20 ms, unsigned-wrap-safe, catch-up burst üretmeyen scheduler.
- Quintic smootherstep ile ortak normalized-time synchronized motion.
- Smootherstep peak slope `1.875` hesaba katılarak servo başına gerçek
  maksimum hız koruması ve otomatik duration extension.
- `REPLACE`: active interpolated point snapshot; eski start'a sıçrama yok.
- `QUEUE`: sabit boyutlu 4-entry FIFO ve atomik full-pose records.
- Single-joint move; diğer joints current/planned pose'u koruyor.
- Per-servo deadband; exact final target write.
- Fixed 192-byte UART line buffer; no `String`, no blocking read,
  overflow/missing/extra/NaN/Inf/range validation.
- Operator-reference `ARM`, `DISARM`, `STOP`, latched `ESTOP`,
  `RESET_ESTOP`, `CLEAR_QUEUE`, `PING`, `STATUS`, `DEMO`.
- 2 s smooth-path communication timeout: current estimate hold, queue clear,
  DISARMED, fresh ARM required.
- Motion completion and timeout events.
- Legacy v1 byte-level responses and direct-P behaviour preserved.
- Smooth `ARM/MOVE/JOINT/NEUTRAL/DEMO/STATUS` angle contract is now explicitly
  ROS joint degrees; measured `zero_offset_rad` is added only inside the
  firmware pulse mapping.
- Calibration anchors, joint limits, zero offsets, inversion and normal speed
  are generated from canonical `servo_calibration.yaml`; firmware build/test
  fails when the checked-in generated header is stale.
- Semantic calibration ID `caf4354aa8ae` is exposed by `PING` and `STATUS`.
- Host-reference versus firmware mapping is compared at five samples per joint
  with an acceptance tolerance of `1 µs`.

## Güvenlik kararları

- **Startup davranışı:** WAIT_FOR_ARMED_COMMAND. Boot PWM off. `ARM` requires
  six observed angles and emits no PWM. First motion begins from that estimate.
- **Unknown position:** firmware never presents the ARM reference as measured
  feedback. Wrong operator reference can still cause first-pulse movement.
- **Timeout davranışı:** active motion stops at current interpolated estimate,
  queue clears, PWM holds, state DISARMED.
- **STOP davranışı:** current interpolated estimate hold; queue cleared; state
  READY.
- **ESTOP davranışı:** motion immediately cancelled, queue cleared, state
  latched. Default PWM HOLD because PWM off can drop the arm. Only
  `RESET_ESTOP` clears the latch, then fresh ARM is required.
- **Physical PWM cut:** legacy `S` remains the explicit PWM-off command; it may
  release a gravity-loaded arm and is not a physical E-stop.
- **Servo limit approach:** invalid smooth angle commands are rejected.
  Mapping functions clamp as a last hardware boundary. Legacy P retains its
  locked clamp semantics for host compatibility.
- **Legacy/smooth ownership:** active legacy PWM cannot be adopted as an angle
  estimate. `S` is required before smooth ARM.

## Build ve test

### Kullanılan build komutu

```bash
cd firmware/esp32_servo_ctrl
pio run -e esp32dev -t clean
pio run -e esp32dev
```

### Build sonucu

- PASS
- RAM: `22,248 / 327,680 bytes` (`6.8%`)
- Flash: `290,609 / 1,310,720 bytes` (`22.2%`)
- New project warnings: none in the clean build output.

### Kullanılan test komutları

```bash
cd firmware/esp32_servo_ctrl
python3 tools/generate_robot_config.py --check
pio test -e native

cd ../stm32_servo_ctrl
pio test -e native
```

### Test sonucu

- Generated calibration freshness: **PASS**, ID `caf4354aa8ae`
- Smooth motion/parser: **22/22 PASS**
- Locked legacy parser: **5/5 PASS**
- `git diff --check`: PASS
- `delay(` in `src/`/`include/`: none
- Arduino `String` in `src/`/`include/`: none

Native coverage includes:

- smootherstep endpoint/monotonicity;
- ROS joint mapping, zero offsets, clamp/reversed;
- six-joint host/firmware PWM parity at five samples per joint (`<=1 µs`);
- slowest-joint and peak-speed duration;
- shared normalized time for all six joints;
- deadband and exact completion;
- replace continuity and FIFO capacity;
- invalid reference/target;
- communication-timeout hold;
- ESTOP latch/reset/rearm;
- valid/invalid/overflow UART parsing.

## UART protokolü

Locked v1 remains:

```text
V?
P500,1000,1500,2000,2500,1750
S
```

Additive smooth commands:

```text
ARM,0,0,0,0,0,0
MOVE,10,-8,5,4,-3,0,2500,REPLACE,42
JOINT,2,-12,1500,QUEUE,43
NEUTRAL,3000,QUEUE,44
STOP
ESTOP
RESET_ESTOP
CLEAR_QUEUE
STATUS
PING
DEMO
```

Full response/state/timeout contract is in
`firmware/esp32_servo_ctrl/README.md`.

## Manuel robot testi

This version was not flashed. Safe physical acceptance order:

1. Servo rail off; arm mechanically supported; physical cutoff in hand.
2. Flash only with explicit user authorization.
3. Servo rail still off: verify `V?`, `PING`, `STATUS=DISARMED/PWM=0`.
4. Logic analyzer: verify boot-low and all six PWM pins.
5. One unloaded servo: measured reference ARM, small slow JOINT move, STOP,
   timeout and ESTOP/reset.
6. One mounted joint: same test within conservative central range; verify
   direction, connector-local voltage and no mechanical load.
7. Six mounted servos: small synchronized MOVE; validate with video plus six
   pin/rail evidence, not UART ACK alone.
8. Queue/replace/link-loss tests only after the small synchronized move passes.

## Açık kalan işler

- No servo-powered motion was done.
- The host `arm_hardware` still uses legacy P by design. Smooth protocol host
  integration is a separate shared-interface decision.
- Every later `servo_calibration.yaml` change must regenerate the firmware
  header; the PlatformIO pre-build check now rejects a stale artifact.
- YAML normal-speed values and firmware-local startup/deadband policies are
  conservative software limits, not measured plant limits.
- Firmware ESTOP cannot replace the physical servo-power cutoff.

## Stage 2 power-off flash and UART acceptance — 2026-07-25

The user confirmed physical readiness with the arm supported and servo rail
off. The existing real-hardware launch was first stopped with `SIGINT`; its
launch, `ros2_control_node` and `robot_state_publisher` processes exited and
`/dev/ttyTHS1` was free before programming.

- Programming port: Jetson `/dev/ttyUSB0`, Silicon Labs CP2102
  (`10c4:ea60`), ESP32D0WDQ6 revision 1.
- Source commit: `95c4915f0cab`.
- Firmware binary: `290976` bytes, SHA-256
  `7e504415a7d041be0b7095ee91fb6977b3cf1cb50baa8147caf6030b747d0659`.
- Four-region flash completed through temporary Python-3.6-compatible
  `esptool 2.8`: bootloader `0x1000`, partitions `0x8000`, boot_app0
  `0xe000`, application `0x10000`.
- Every written region reported `Hash of data verified`; hard reset completed.
- Power-off UART2 acceptance:
  - `V? -> V1,1.2.0-esp32`
  - `PING -> ACK,PING,FW=1.2.0-esp32,FRAME=ROS_JOINT_DEG,CALIB=caf4354aa8ae`
  - `STATUS -> STATE=DISARMED, MOVING=0, QUEUE=0, PWM=0`
  - all current/target ROS joint angles `0.000`, all moving flags `0`
- Negative gate: a six-zero `MOVE` while DISARMED returned `ERR,NOT_READY`;
  the following STATUS remained `DISARMED`, `PWM=0`.

This is a real flash and UART acceptance, but still not PWM-pin or actuator
acceptance. Servo power stayed off. No `ARM`, legacy `P`, `NEUTRAL`, `DEMO` or
servo-powered command was sent. Remaining order: six-pin boot-low/PWM capture,
one unloaded servo, one supported mounted joint, then a small synchronized
six-joint move.

### Immediate timeout defect found and closed

The first power-off ARM attempt exposed a real timing defect before actuator
power was enabled:

```text
ACK,ARM,PWM=OFF
EVENT,COMM_TIMEOUT,STATE=DISARMED,PWM=HOLD
```

Root cause: `loop()` sampled `nowMs` before RX processing, while an accepted
command recorded `lastSmoothCommandMs=millis()` during RX. If the millisecond
tick advanced inside parsing, unsigned `nowMs-lastSmoothCommandMs` underflowed
to a huge duration and falsely triggered timeout in the same loop. The same
ordering risk also existed in the legacy watchdog timestamp.

Fix in firmware `1.2.1-esp32`:

- sample the loop timestamp after RX processing;
- use a wrap-safe half-range `elapsedAtLeast()` helper that rejects a
  just-recorded future timestamp;
- add native regression coverage for future-sample rejection, normal deadline
  crossing and `millis()` wraparound.

Verification:

- native tests **23/23 PASS**;
- clean ESP32 build PASS, RAM 6.8%, flash 22.2%;
- application SHA-256
  `edeab2cb27a912487f9a5081acbf8dec9876936a01446a0b3254178494f78f96`;
- four-region reflash: every region hash-verified;
- `V? -> V1,1.2.1-esp32`;
- ARM remained `READY/PWM=0` after 0.5 s and again after a later PING/STATUS;
- explicit DISARM returned to `DISARMED/PWM=0`.

Servo power remained off throughout defect reproduction, fix reflash and
revalidation. No physical movement occurred.

## Stage 3 mounted-arm motion and PWM quantization — 2026-07-25

At the user's direction, previously completed analyzer and single-servo gates
were not repeated. With all six mounted servos powered and a physical cutoff
available, the following synchronized moves completed without a firmware
fault:

- ROS zero reference, then `[2,-2,2,2,-2,1] deg` over `3000 ms`;
- `[2,-2,2,2,-2,1]` to `[8,-8,8,8,-8,4] deg` over `5000 ms`;
- `[8,-8,8,8,-8,4]` to `[25,-25,12,20,-20,8] deg` over `6000 ms`.

The user visually confirmed that the arm reached zero and that the six-joint
motion was synchronized, but reported that the physical motion was still not
smooth.

An offline replay of the final 300 scheduler intervals found that the quintic
joint profile was being mapped to an integer microsecond before conversion to
16-bit LEDC duty. This discarded the ESP32's approximately `0.305 us` PWM
resolution. Repeated-output interval counts for joints 1 through 6 were
`122,121,256,167,167,104` with the integer-microsecond path. Direct
floating-point pulse-to-duty conversion reduces them to
`57,57,154,70,70,52`, respectively. Distinct command levels increase from
`179,180,45,134,134,197` to `244,244,147,231,231,249`.

Firmware `1.2.2-esp32` therefore preserves the continuous pulse calculation
until the final LEDC-count rounding. The locked legacy direct-`P` path keeps
its existing integer-microsecond conversion unchanged. Verification before
flash:

- smooth native tests **24/24 PASS**, including a regression where two joint
  angles map to the same integer microsecond but different LEDC duty counts;
- locked legacy parser **5/5 PASS**;
- clean ESP32 build PASS, RAM `6.8%`, flash `22.2%`;
- application binary SHA-256
  `f9cef378fa01c387ed9aa6a61b863097cc31b96a555ef1f729182a721a2368f7`;
- `git diff --check` PASS.

Physical reflash and comparison motion remain pending. Flashing resets the
ESP32 and disables PWM, so the servo rail must first be turned off and the arm
mechanically supported.

### Stage 3 reflash and visual comparison result

After the user confirmed the servo rail was off, firmware `1.2.2-esp32` was
flashed through Jetson `/dev/ttyUSB0`. Bootloader, partition table, boot_app0
and application regions each reported `Hash of data verified`; hard reset
completed. With servo power still off, UART2 acceptance returned:

- `V1,1.2.2-esp32`;
- frame `ROS_JOINT_DEG`;
- calibration ID `caf4354aa8ae`;
- `DISARMED`, `PWM=0`, no queued or active motion;
- explicit DISARM retained `PWM=0`.

The user then confirmed servo power on and no physical pose change. Because
the ESP32 reset had erased its open-loop state, ARM used the last visually
confirmed pose `[25,-25,12,20,-20,8] deg`, not firmware zero.

Two same-path comparisons isolated the visual-smoothness boundary:

1. `[25,-25,12,20,-20,8] -> [8,-8,8,8,-8,4] deg`, `6000 ms`:
   completed without a fault, but the user reported that it looked the same
   as the integer-microsecond firmware.
2. Reverse path, `1500 ms`: completed without a fault; the user reported
   **"gayet iyi"**.

For the 17-degree main-joint delta, the 6-second profile changes about
`1.18 us` per 20 ms at peak. The 1.5-second profile changes about `4.72 us`
per 20 ms at peak, with peak joint speed `21.25 deg/s`, below the configured
`22.918 deg/s` limit. The sub-microsecond firmware fix improves command
fidelity but did not visibly change the mechanism by itself.

Conclusion: the quintic synchronization and 50 Hz scheduler are functioning;
the dominant visible stepping occurs when the requested motion is slow enough
to remain inside the hobby servo's effective deadband/static-friction region.
The exact split among servo electronics, gearbox friction and loaded-joint
mechanics is not instrumented. For this tested path, visually smooth planning
should target roughly `15-21 deg/s` peak on the dominant joints while retaining
the configured hard maximum. A universal minimum speed should not be hardcoded
from one path; host integration should select duration from joint delta and a
configurable visual-speed target.
