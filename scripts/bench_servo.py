#!/usr/bin/env python3
"""Bench-drive the six Robot Arm servos straight over UART protocol v1.

No ROS, no pyserial: this exists to answer one question on free-standing servos
-- does a servo SETTLE at a fixed pulse (positional) or keep turning
(continuous rotation)?  It deliberately does not import anything from the
workspace, so nothing here can disturb arm_hardware / controller config.

Protocol (firmware/stm32_servo_ctrl/README.md):
    V?\\n                    -> V1,...
    P<us>,<us>,...(6)\\n     -> OK        (all six channels mandatory; E2 otherwise)
    S\\n                     -> all PWM off

Streams the frame at 20 Hz because the firmware emits E3 once after 1000 ms with
no position command, which would otherwise pollute the response line.
"""

import argparse
import os
import select
import sys
import termios
import time

CHANNEL_PINS = ["PA0", "PA1", "PA2", "PA3", "PA6", "PA7"]
CHANNEL_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6/gripper"]


def open_port(device):
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    cc = list(attrs[6])
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(
        fd,
        termios.TCSANOW,
        [0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0,
         termios.B115200, termios.B115200, cc],
    )
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_line(fd, timeout_s):
    deadline = time.time() + timeout_s
    buf = bytearray()
    while time.time() < deadline:
        ready, _, _ = select.select([fd], [], [], max(0.0, deadline - time.time()))
        if not ready:
            break
        chunk = os.read(fd, 64)
        if not chunk:
            continue
        for byte in chunk:
            if byte in (0x0A, 0x0D):
                if buf:
                    return buf.decode("ascii", "replace").strip()
            else:
                buf.append(byte)
    return buf.decode("ascii", "replace").strip() if buf else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/ttyTHS1")
    parser.add_argument("--pulses", required=True,
                        help="one value for all six, or six comma-separated values")
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--stop-after", action="store_true",
                        help="send S (PWM off) when the hold window ends")
    args = parser.parse_args()

    fields = [f.strip() for f in args.pulses.split(",")]
    pulses = [int(fields[0])] * 6 if len(fields) == 1 else [int(f) for f in fields]
    if len(pulses) != 6:
        sys.exit("need exactly 1 or 6 pulse values")
    for value in pulses:
        if not 500 <= value <= 2500:
            sys.exit(f"pulse {value} outside the 500-2500 us the calibration allows")

    fd = open_port(args.device)
    try:
        os.write(fd, b"V?\n")
        version = read_line(fd, 1.0)
        if version is None or not version.startswith("V1,"):
            sys.exit(f"handshake failed, firmware said {version!r} (expected 'V1,...')")
        print(f"handshake OK: {version}")

        frame = ("P" + ",".join(str(p) for p in pulses) + "\n").encode()
        print("holding:")
        for index, value in enumerate(pulses):
            print(f"  ch{index + 1}  {CHANNEL_PINS[index]:>4}  "
                  f"{CHANNEL_JOINTS[index]:<16} {value} us")
        print(f"streaming at 20 Hz for {args.seconds:.0f} s -- WATCH THE HORNS\n")

        ok = 0
        errors = {}
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            os.write(fd, frame)
            reply = read_line(fd, 0.05)
            if reply == "OK":
                ok += 1
            elif reply is not None:
                errors[reply] = errors.get(reply, 0) + 1
            time.sleep(0.05)

        print(f"frames acknowledged: {ok}")
        print(f"unexpected replies : {errors if errors else 'none'}")

        if args.stop_after:
            os.write(fd, b"S\n")
            print(f"S sent (PWM off), reply: {read_line(fd, 0.5)!r}")
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
