# STM32 servo controller

PlatformIO firmware for the Blue Pill board driving six 50 Hz servo PWM outputs.
It implements the UART protocol v1 documented below.

Everything below was verified end-to-end on hardware on 2026-07-14.

## Chip

The part is **medium-density with 128 KB flash / 20 KB RAM**, not the C6 this
project previously assumed. Read from the chip itself: bootloader `GET ID`
returns PID `0x0410`, and the flash-size register at `0x1FFFF7E0` reports 128 KB.
`platformio.ini` targets `bluepill_f103c8` accordingly.

It is a **GD32F103 clone** (ROM bootloader reports v3.0/`0x30`; genuine ST parts
report v2.2). This matters — see the USB section.

## PWM pin map (verified)

| Protocol channel | URDF joint | Timer channel | Pin |
| --- | --- | --- | --- |
| 1 | joint_1 | TIM2_CH1 | PA0 |
| 2 | joint_2 | TIM2_CH2 | PA1 |
| 3 | joint_3 | TIM4_CH1 | **PB6** (PA2 until 0.5.0, PB0 in 0.5.0 only) |
| 4 | joint_4 | TIM2_CH4 | PA3 |
| 5 | joint_5 | TIM3_CH1 | PA6 |
| 6 | joint_6 (gripper) | TIM3_CH2 | PA7 |

**PA2 and PB0 are abandoned, not spare — do not reuse either.** Both were
measured dead with a logic analyzer on 2026-07-15, and the analyzer channel was
itself validated against a known-good pin in the same session before the verdict
was accepted:

| Timer channel | Pin | Commanded | Measured |
| --- | --- | --- | --- |
| TIM2_CH1 | PA0 | 500 us | 500.1 us @ 50.0 Hz |
| TIM2_CH2 | PA1 | 1500 us | 1500.4 us @ 50.0 Hz |
| TIM3_CH1 | PA6 | 1000 us | 1000.3 us @ 50.0 Hz |
| TIM4_CH1 | PB6 | 2500 us | 2500.7 us @ 50.0 Hz |
| **TIM2_CH3** | PA2 | — | **dead** |
| **TIM3_CH3** | PB0 | 2500 us | **zero edges in 200k samples** |

`CHANNEL_3` is dead on two independent timers while `CHANNEL_1` works on three.
The root cause is **NOT established**. The code path was read end to end —
parser (unit tested), `apply_positions`, `__HAL_TIM_SET_COMPARE`,
`HAL_TIM_PWM_Start`, `TIM_CCxChannelCmd` — and every step is channel-agnostic,
so a code fault does not explain it. Two physically separate dead pins does not
explain it either. Moving to TIM4_CH1/PB6 is an **avoidance, not a fix**: the
mechanism is still unknown and could resurface.

An `R` command was added for this: it dumps CCER/CCR/ARR/PSC straight from the
chip, so the next person can ask the silicon what its registers hold instead of
inferring backwards from how a servo behaves. That inference wasted hours here
and produced three confident, wrong diagnoses in a row.

Channel↔joint mapping lives in
`src/robot_arm_description/config/servo_calibration.yaml` (1-indexed `channel:`
there; 0-indexed `pulse_us[]` here).

PWM timers run from a 1 MHz counter with period 20000, so the compare value
equals pulse width in microseconds directly. Outputs stay disabled at boot until
the first valid position command. `S` disables all PWM outputs. After 1000 ms
with no position command the last values remain active and the firmware reports
`E3` once.

## Transport: UART, not USB

Protocol runs over **USART1 — PA9 (TX) / PA10 (RX), 115200 8N1**, wired to the
Jetson Nano's own 40-pin header UART. No FTDI adapter is involved:

| Nano 40-pin | STM32 |
| --- | --- |
| pin 8 (UART TX) | PA10 |
| pin 10 (UART RX) | PA9 |

Ground is already shared through the USB power cable, so no extra GND jumper is
needed. Port on the Nano is `/dev/ttyTHS1`; it needs `sudo systemctl disable
nvgetty` (otherwise a serial console holds it) and a udev rule for permissions:

```
KERNEL=="ttyTHS1", MODE="0660", GROUP="dialout"
```

**Do not print anything but protocol on this line.** `stm32_system_interface.cpp`
compares responses against exactly `"OK"`; one stray debug line breaks the
handshake. Diagnostics go to the PC13 LED instead: 1 Hz blink = loop alive,
solid = hung, N-pulse groups = `fatal_error(N)`.

### USB CDC does not work on this chip

Confirmed with usbmon. The device enumerates (`/dev/ttyACM0`, `cdc_acm` binds)
and **control transfers work** — the firmware sees DTR assert. But **both bulk
endpoints NAK forever**: the host submits the data correctly
(`S Bo:1:011:1 -115 2 = 560a` for `V\n`), no URB ever completes, and they are all
cancelled with `-2` (ENOENT) when the port closes. Bulk IN behaves the same.

This rules out the usual suspects — dead D-, wrong clock, charge-only cable —
since enumeration proves those all work. It is a GD32-vs-ST USB IP difference at
the bulk endpoint level and cannot be fixed with build flags. The USB stack is
therefore compiled out entirely; the micro-USB port is **power only**.

If a single-cable-to-Nano setup is wanted, use a CH340/FTDI module (one USB cable
to the Nano, three wires to PA9/PA10/GND) or a genuine STM32 part.

## Protocol quick reference

| Send | Reply |
| --- | --- |
| `V?` | `V1,<version>` |
| `P<us>,<us>,<us>,<us>,<us>,<us>` | `OK` |
| `S` | `OK` |
| malformed | `E1` |
| wrong channel count | `E2` |
| 1 s without a position command | `E3` (once; last PWM retained) |

Note it is `V?`, not `V` — plain `V` returns `E1`.

## Flashing

The F103 ROM bootloader supports **USART only — no USB DFU**. There is no ST-Link
in this setup, so flashing goes through the Nano's UART:

1. `BOOT0` = **1**, press RESET
2. `stm32flash -b 115200 -w .pio/build/bluepill_f103c8/firmware.bin -v /dev/ttyTHS1`
3. `BOOT0` = **0**, press RESET

`-b 115200` is required: the bootloader locks its baud on the first `0x7F` sync
byte, and stm32flash defaults to 57600, which then fails with `Failed to init
device`. The bootloader speaks **8E1** while the firmware speaks **8N1** —
remember to switch parity when poking the line by hand.

Liveness check that bypasses flash entirely (BOOT0=1 + RESET, then send `0x7F`):
a `0x79` ACK proves the chip, its PA9/PA10 solder joints, and the wiring are all
good.

## The SysTick trap

**Use `framework = arduino`.** Under bare-metal `framework = stm32cube` you must
define `SysTick_Handler` yourself; if you don't, the startup file's weak symbol
resolves to `Default_Handler` → `Infinite_Loop` and **the chip freezes ~1 ms into
every boot** — solid LED, silent UART, no response to anything. This bit the
project twice: fixed on 2026-07-11, then silently reintroduced when the UART
fallback was restored from an older `main.c` that predated the fix.

The Arduino core supplies `SysTick_Handler`, which closes the trap structurally.
Verify after any framework change:

```bash
arm-none-eabi-nm firmware.elf | grep -E "SysTick_Handler|Default_Handler"
```

Same address for both = broken.

## Build and test without flashing

```bash
pio run
pio test -e native
```
