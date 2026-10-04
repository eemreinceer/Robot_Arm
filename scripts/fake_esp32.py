#!/usr/bin/env python3
"""Fake ESP32 speaking locked UART v1, used to prove the host error path.

It answers V? and the first N position frames normally, then goes SILENT so the
host's ACK window expires. Everything received afterwards is recorded -- in
particular whether the `S` stop frame arrives, which is the whole question:

  does ros2_control actually route a write() ERROR into on_error, and does
  on_error actually put the stop frame on the wire?

The unit test could only show "if error() is called, S is sent". This shows the
full chain through the real resource manager.
"""
import argparse
import json
import os
import pty
import sys
import time

parser = argparse.ArgumentParser()
parser.add_argument("--device-file", required=True,
                    help="file to write the slave tty path into, for the host")
parser.add_argument("--answer-count", type=int, default=10,
                    help="position frames to ACK before going silent")
parser.add_argument("--report", required=True)
parser.add_argument("--duration", type=float, default=60.0)
parser.add_argument("--calibration-fingerprint", default="21aecfe95643057d",
                    help="answered to K?; 'unsupported' replies E1 like an "
                         "older build, any other value stands in for a firmware "
                         "built from a different calibration YAML")
parser.add_argument("--fault-mode", choices=("silence", "watchdog-e3"),
                    default="silence",
                    help="silence: stop answering, so the host's ACK window "
                         "expires. watchdog-e3: answer E3 instead of OK, which "
                         "is what the real firmware puts on the wire when its "
                         "own watchdog fires (policy C, firmware >= 1.4.0). The "
                         "host must treat that as a fault too, and it reaches "
                         "the host as a REPLY rather than as a timeout, so it "
                         "exercises a different branch of write().")
args = parser.parse_args()

# Own the master end; the host opens the slave path as if it were /dev/ttyTHS1.
fd, slave_fd = pty.openpty()
slave_path = os.ttyname(slave_fd)
with open(args.device_file, "w") as handle:
    handle.write(slave_path)
print("fake ESP32 listening, host device = %s" % slave_path, flush=True)
os.set_blocking(fd, False)

state = {
    "fault_mode": args.fault_mode,
    "version_requests": 0,
    "fingerprint_requests": 0,
    "position_frames": 0,
    "acked_frames": 0,
    "silent_frames": 0,
    "e3_frames": 0,
    "stop_frames_after_silence": 0,
    "stop_frames_total": 0,
    "first_stop_after_silence_s": None,
    "unknown": [],
    "went_silent": False,
    # Mode-independent view of the same question: once the fault started, did
    # the stop frame arrive? The silence-specific keys above are kept so older
    # callers and recorded artifacts keep meaning what they meant.
    "fault_started": False,
    "stop_frames_after_fault": 0,
    "first_stop_after_fault_s": None,
}

buffer = b""
started = time.monotonic()
silence_started = None
fault_started_at = None


def write_report():
    with open(args.report, "w") as handle:
        json.dump(state, handle, indent=2)


while time.monotonic() - started < args.duration:
    try:
        chunk = os.read(fd, 256)
    except BlockingIOError:
        time.sleep(0.001)
        continue
    except OSError:
        time.sleep(0.01)
        continue
    if not chunk:
        time.sleep(0.001)
        continue
    buffer += chunk
    while b"\n" in buffer:
        line, buffer = buffer.split(b"\n", 1)
        text = line.decode("ascii", "replace").strip()
        if not text:
            continue
        if text.startswith("V?"):
            state["version_requests"] += 1
            os.write(fd, b"V1,1.2.2-esp32\n")
        elif text.startswith("K?"):
            # Calibration fingerprint. --calibration-fingerprint stands in for a
            # firmware built from a different YAML, which is the case the host's
            # activation gate has to refuse; the default answers with whatever
            # the host expects so the ACK tests are unaffected by the gate.
            state["fingerprint_requests"] += 1
            if args.calibration_fingerprint == "unsupported":
                os.write(fd, b"E1\n")
            else:
                os.write(fd, f"K,{args.calibration_fingerprint}\n".encode("ascii"))
        elif text.startswith("P"):
            state["position_frames"] += 1
            if state["position_frames"] <= args.answer_count:
                state["acked_frames"] += 1
                os.write(fd, b"OK\n")
            else:
                # The deliberate fault under test.
                if not state["fault_started"]:
                    state["fault_started"] = True
                    fault_started_at = time.monotonic()
                if args.fault_mode == "silence":
                    state["silent_frames"] += 1
                    if not state["went_silent"]:
                        state["went_silent"] = True
                        silence_started = fault_started_at
                else:
                    # What firmware >= 1.4.0 actually emits when its legacy
                    # watchdog expires. It arrives as a reply, not as a missing
                    # one, so the host sees a completed line that is not "OK".
                    state["e3_frames"] += 1
                    os.write(fd, b"E3\n")
        elif text.startswith("S"):
            state["stop_frames_total"] += 1
            if state["fault_started"]:
                state["stop_frames_after_fault"] += 1
                if state["first_stop_after_fault_s"] is None and fault_started_at:
                    state["first_stop_after_fault_s"] = round(
                        time.monotonic() - fault_started_at, 4)
            if state["went_silent"]:
                state["stop_frames_after_silence"] += 1
                if state["first_stop_after_silence_s"] is None and silence_started:
                    state["first_stop_after_silence_s"] = round(
                        time.monotonic() - silence_started, 4)
        else:
            state["unknown"].append(text[:40])
        write_report()

write_report()
print(json.dumps(state, indent=2))
sys.exit(0)
