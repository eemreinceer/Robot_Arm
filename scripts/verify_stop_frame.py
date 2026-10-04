#!/usr/bin/env python3
"""Does the real ESP32 drop PWM when it receives the locked v1 `S` frame?

CONFOUND THIS GUARDS AGAINST: the firmware has a ~1 s command watchdog. Send one
P, wait, and PWM goes to 0 all by itself -- which would look like a pass while
proving nothing about `S`. So the position frame is FED continuously to keep the
watchdog satisfied, PWM=1 is confirmed while feeding, and then STATUS is read
within milliseconds of `S` -- far inside the watchdog window.

SAFETY: run this ONLY with the servo rail off and the arm supported. The
commanded vector is the documented ROS q=0 pulse set and the firmware reports
CURRENT=0 for all joints, so the commanded delta is zero -- but this DOES drive
PWM, so treat it as a powered-path test, not a read-only probe.

Run inside the container that owns the UART:
    docker exec -i robot_arm_hw python3 - < scripts/verify_stop_frame.py

MEASURED 2026-07-29 (real ESP32 1.2.2-esp32, servo rail off, two runs):
    PWM 1 while fed -> 0 within 401 ms (coarse read) and 24.6 ms (tight read).
    The watchdog is ~1000 ms, so `S` is what cleared PWM, not the timeout.

FINDING worth remembering: PWM reached 1 while the firmware still reported
STATE=DISARMED. The locked legacy `P` path drives PWM regardless of the smooth
protocol's armed state, so "STATE=DISARMED" in a preflight means "not armed for
smooth motion" -- it does NOT mean PWM is inhibited. Read PWM= explicitly.
"""
import os
import sys
import termios
import time

DEV = "/dev/ttyTHS1"
# Documented ROS q=0 vector (STATUS board, calib caf4354aa8ae).
Q0_US = [1500, 1914, 2290, 1373, 1436, 1500]
P_FRAME = ("P" + ",".join(str(v) for v in Q0_US) + "\n").encode()

fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
attrs = termios.tcgetattr(fd)
cc = list(attrs[6])
cc[termios.VMIN] = 0
cc[termios.VTIME] = 0
termios.tcsetattr(fd, termios.TCSANOW, [
    0, 0, termios.CS8 | termios.CREAD | termios.CLOCAL, 0,
    termios.B115200, termios.B115200, cc])
termios.tcflush(fd, termios.TCIOFLUSH)


def drain(wait):
    deadline = time.monotonic() + wait
    buf = b""
    while time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 512)
        except (BlockingIOError, OSError):
            time.sleep(0.002)
            continue
        if chunk:
            buf += chunk
        else:
            time.sleep(0.002)
    return [l for l in buf.decode("ascii", "replace").splitlines() if l.strip()]


def status(wait=0.6):
    termios.tcflush(fd, termios.TCIFLUSH)
    os.write(fd, b"STATUS\n")
    deadline = time.monotonic() + wait
    buf = b""
    while time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 512)
        except (BlockingIOError, OSError):
            time.sleep(0.001)
            continue
        if chunk:
            buf += chunk
            text = buf.decode("ascii", "replace")
            for line in text.splitlines():
                if line.startswith("STATE") and "\n" in text:
                    return line
        else:
            time.sleep(0.001)
    lines = [l for l in buf.decode("ascii", "replace").splitlines() if l.strip()]
    for line in lines:
        if line.startswith("STATE"):
            return line
    return "(no STATE line: %s)" % lines


def pwm_of(state_line):
    for field in state_line.split(","):
        if field.startswith("PWM="):
            return field.split("=", 1)[1]
    return "?"


results = {}
try:
    print("baseline           : %s" % status())

    print("feeding P frames to keep the watchdog satisfied...")
    feed_until = time.monotonic() + 2.0
    while time.monotonic() < feed_until:
        os.write(fd, P_FRAME)
        time.sleep(0.02)
        drain(0.0)

    # Confirm PWM actually came up WHILE feeding; without this the later
    # PWM=0 would be unfalsifiable.
    os.write(fd, P_FRAME)
    fed_state = status(0.4)
    results["pwm_while_fed"] = pwm_of(fed_state)
    print("while fed          : %s" % fed_state)

    # Keep feeding right up to the stop frame so the watchdog cannot be the
    # thing that clears PWM.
    for _ in range(10):
        os.write(fd, P_FRAME)
        time.sleep(0.02)
        drain(0.0)

    last_feed = time.monotonic()
    os.write(fd, b"S\n")
    sent_at = time.monotonic()
    after_state = status(0.4)
    elapsed_ms = (time.monotonic() - last_feed) * 1000.0
    results["pwm_after_stop"] = pwm_of(after_state)
    results["ms_since_last_feed"] = round(elapsed_ms, 1)
    results["ms_stop_to_status"] = round((time.monotonic() - sent_at) * 1000.0, 1)
    print("after S            : %s" % after_state)
    print("elapsed since last P frame: %.1f ms (watchdog is ~1000 ms)" % elapsed_ms)
finally:
    # Always leave the MCU stopped, whatever happened above.
    os.write(fd, b"S\n")
    time.sleep(0.2)
    final = status(0.6)
    print("final              : %s" % final)
    results["pwm_final"] = pwm_of(final)
    os.close(fd)

print()
print("RESULT: %s" % results)
fed = results.get("pwm_while_fed")
after = results.get("pwm_after_stop")
ms = results.get("ms_since_last_feed", 9999)
if fed == "1" and after == "0" and ms < 500:
    print("PASS: PWM was 1 while fed and 0 within %.1f ms of S -- too fast for the "
          "watchdog, so `S` did it." % ms)
    sys.exit(0)
if fed != "1":
    print("INCONCLUSIVE: PWM never reached 1 while feeding (got %r); the stop frame "
          "had nothing to clear. Needs a different way to raise PWM." % fed)
    sys.exit(2)
print("FAIL: PWM did not clear on S (fed=%r after=%r)." % (fed, after))
sys.exit(1)
