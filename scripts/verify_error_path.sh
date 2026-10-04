#!/usr/bin/env bash
#
# Proves the host fails CLOSED when the MCU stops answering.
#
# The unit test (arm_hardware/test/test_error_transition.cpp) can only show
# "if on_error is called, the stop frame is sent". It calls the lifecycle
# transition directly. The open question was the link before that:
#
#   does ros2_control actually route a write() ERROR into on_error?
#
# This script answers it with the real controller_manager and a fake ESP32 that
# speaks locked UART v1 and then goes deliberately silent. The proof is taken
# from the MCU side of the wire, not from host logs: scripts/fake_esp32.py
# records whether `S` arrives after the fault.
#
# NO PHYSICAL HARDWARE IS INVOLVED -- the serial link is a pty. Safe to run on a
# laptop or on the Jetson without touching the arm or the servo rail.
#
# Measured 2026-07-29 (laptop, Jazzy):
#   acked_frames 15, silent_frames 1, stop_frames_after_silence 1,
#   first_stop_after_silence_s 0.0258, hardware active -> unconfigured.
#
# Two faults can be injected; both must fail CLOSED.
#   silence      the MCU stops answering, so the ACK window expires (default)
#   watchdog-e3  the MCU answers E3 -- what firmware >= 1.4.0 emits when its own
#                watchdog fires (policy C). This arrives as a completed reply
#                rather than as a timeout, so it takes a different branch of
#                write(), and it was NOT covered until 2026-08-15.
#
# Usage: verify_error_path.sh [silence|watchdog-e3|both]
#
# NOTE: no `set -u` -- the ROS setup scripts reference unbound variables.
set -o pipefail

MODE="${1:-both}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="${TMPDIR:-/tmp}/robot_arm_error_path_$$"
mkdir -p "$WORK"

REPORT_DIR="$WORK"

cleanup() {
  [ -n "${CM_PID:-}" ] && kill "$CM_PID" 2>/dev/null
  [ -n "${FAKE_PID:-}" ] && kill "$FAKE_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "Source your ROS setup first (e.g. /opt/ros/jazzy/setup.bash) and the workspace install." >&2
  exit 1
fi

run_case() {
  FAULT="$1"
  REPORT="$REPORT_DIR/fake_esp32_report_$FAULT.json"
  CM_LOG="$REPORT_DIR/ros2_control_$FAULT.log"
  DEVFILE="$REPORT_DIR/host_device_path_$FAULT"

  echo
  echo "############ fault: $FAULT ############"
  echo "== starting fake ESP32 =="
  python3 "$HERE/fake_esp32.py" --device-file "$DEVFILE" --answer-count 15 \
    --fault-mode "$FAULT" \
    --report "$REPORT" --duration 70 >"$REPORT_DIR/fake_$FAULT.log" 2>&1 &
  FAKE_PID=$!
  for _ in $(seq 1 50); do [ -s "$DEVFILE" ] && break; sleep 0.1; done
  if [ ! -s "$DEVFILE" ]; then
    echo "fake ESP32 failed to start:" >&2
    cat "$REPORT_DIR/fake_$FAULT.log" >&2
    return 1
  fi
  FAKE_TTY="$(cat "$DEVFILE")"
  export FAKE_TTY
  echo "   host device = $FAKE_TTY"

  echo "== launching real ros2_control stack =="
  ros2 launch "$HERE/error_path.launch.py" >"$CM_LOG" 2>&1 &
  CM_PID=$!
  for _ in $(seq 1 60); do
    ros2 node list 2>/dev/null | grep -q controller_manager && break
    sleep 0.5
  done
  sleep 3

  echo "== hardware state before =="
  ros2 control list_hardware_components 2>/dev/null | head -3

  echo "== arming at zero reference =="
  ros2 param set /robot_arm_hardware_safety reference_positions \
    "[0.0,0.0,0.0,0.0,0.0,0.0]" >/dev/null 2>&1
  ros2 param set /robot_arm_hardware_safety armed true >/dev/null 2>&1

  echo "== running into the deliberate fault =="
  sleep 10

  echo "== hardware state after =="
  ros2 control list_hardware_components 2>/dev/null | head -3

  kill "$CM_PID" 2>/dev/null; wait "$CM_PID" 2>/dev/null
  CM_PID=""
  sleep 1
  kill "$FAKE_PID" 2>/dev/null; wait "$FAKE_PID" 2>/dev/null
  FAKE_PID=""

  echo
  echo "== host log (error path) =="
  grep -E "ACK timed out|watchdog fired|error transition|ACK summary|Discarded" "$CM_LOG" | sed 's/^/   /'

  echo
  echo "== MCU-side evidence =="
  cat "$REPORT"

  STOPS=$(python3 -c "import json; print(json.load(open('$REPORT'))['stop_frames_after_fault'])" 2>/dev/null)
  FAULTED=$(python3 -c "import json; print(json.load(open('$REPORT'))['fault_started'])" 2>/dev/null)
  echo
  if [ "$FAULTED" = "True" ] && [ "${STOPS:-0}" -ge 1 ]; then
    echo "PASS ($FAULT): the MCU received the stop frame after the fault (fail-closed)."
    return 0
  fi
  echo "FAIL ($FAULT): no stop frame reached the MCU after the fault (fail-OPEN)."
  return 1
}

case "$MODE" in
  silence|watchdog-e3) CASES="$MODE" ;;
  both) CASES="silence watchdog-e3" ;;
  *) echo "usage: $0 [silence|watchdog-e3|both]" >&2; exit 2 ;;
esac

RC=0
for case_name in $CASES; do
  run_case "$case_name" || RC=1
done

echo
echo "artifacts: $WORK"
exit "$RC"
