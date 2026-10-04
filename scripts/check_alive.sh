#!/usr/bin/env bash
#
# Liveness check that trusts DATA, not listings.
#
# Twice on 2026-07-29 a listing lied and cost real time:
#
#   * `/camera/image_raw` appeared in `ros2 topic list` with NO publisher --
#     mjpeg_bridge was merely subscribed to it. The camera node had been dead
#     for 19 hours and nobody noticed.
#   * `ros2 node list` showed /robot_state_publisher and /controller_manager
#     after the stack had died, because robot_state_publisher segfaults on
#     shutdown under CycloneDDS on the Nano and leaves ghost discovery entries.
#     The arm stack looked alive; TF lookup proved it was not.
#
# So every check here either measures a rate, resolves a transform, or calls a
# service. Presence in a list is reported only to CONTRAST it with reality.
#
# Usage:
#   scripts/check_alive.sh              # camera + arm
#   scripts/check_alive.sh --camera     # camera only (arm stack intentionally down)
#
# NOTE: no `set -u` -- the ROS setup scripts reference unbound variables.

CHECK_CAMERA=1
CHECK_ARM=1
case "${1:-}" in
  --camera) CHECK_ARM=0 ;;
  --arm)    CHECK_CAMERA=0 ;;
  --help|-h)
    echo "usage: $0 [--camera|--arm]"; exit 0 ;;
esac

BASE_FRAME="${BASE_FRAME:-base_link}"
WRIST_FRAME="${WRIST_FRAME:-link_5}"
failures=0

pass() { printf "  \033[32mPASS\033[0m  %s\n" "$1"; }
fail() { printf "  \033[31mFAIL\033[0m  %s\n" "$1"; failures=$((failures+1)); }
info() { printf "        %s\n" "$1"; }

if [ -z "${ROS_DISTRO:-}" ]; then
  echo "Source a ROS setup first (e.g. /opt/ros/jazzy/setup.bash)." >&2
  exit 2
fi

# The Nano containers embed CycloneDDS. A PC session that does not set the same
# RMW sees an EMPTY graph and every check below fails for the wrong reason --
# measured during the Phase 15 investigation. Catch that first.
echo "== environment =="
if [ -z "${RMW_IMPLEMENTATION:-}" ]; then
  fail "RMW_IMPLEMENTATION unset -- the Nano uses CycloneDDS; a mismatched PC
        session sees nothing and every check below would fail misleadingly.
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
else
  pass "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
fi

# Reports the measured rate, and separately the publisher count, so the
# "listed but nobody publishes" case is named explicitly instead of looking
# like a slow topic.
check_topic() {
  local topic="$1" min_hz="$2" window="${3:-15}"
  local pubs rate
  pubs=$(timeout 8 ros2 topic info "$topic" 2>/dev/null \
         | awk -F': ' '/Publisher count/{print $2}')
  pubs=${pubs:-0}
  rate=$(timeout "$window" ros2 topic hz "$topic" --window 20 2>/dev/null \
         | awk '/average rate/{print $3; exit}')
  if [ "$pubs" = "0" ]; then
    fail "$topic -- listed but PUBLISHER COUNT 0 (a subscriber alone keeps a
        topic visible; the publisher is dead)"
    return
  fi
  if [ -z "$rate" ]; then
    fail "$topic -- $pubs publisher(s) advertised but NO MESSAGES arrived"
    return
  fi
  if awk "BEGIN{exit !($rate >= $min_hz)}"; then
    pass "$topic  ${rate} Hz  (${pubs} publisher)"
  else
    fail "$topic  only ${rate} Hz, expected >= ${min_hz}"
  fi
}

if [ "$CHECK_CAMERA" = "1" ]; then
  echo "== camera (measured, not listed) =="
  check_topic /camera/image_raw 20
  check_topic /camera/camera_info 20
fi

if [ "$CHECK_ARM" = "1" ]; then
  echo "== arm stack (measured, not listed) =="
  # TF resolution is the honest liveness test for robot_state_publisher: ghost
  # discovery entries cannot answer a transform query.
  if timeout 12 ros2 run tf2_ros tf2_echo "$BASE_FRAME" "$WRIST_FRAME" 2>/dev/null \
       | grep -q "Translation"; then
    pass "TF $BASE_FRAME -> $WRIST_FRAME resolves"
  else
    fail "TF $BASE_FRAME -> $WRIST_FRAME does NOT resolve -- robot_state_publisher
        is not actually publishing, whatever 'ros2 node list' claims"
  fi

  # A service call needs a live server; discovery ghosts cannot answer one.
  if hw=$(timeout 12 ros2 control list_hardware_components 2>/dev/null) && [ -n "$hw" ]; then
    state=$(echo "$hw" | awk -F'label=' '/label=/{print $2; exit}')
    pass "controller_manager answered; hardware state: ${state:-unknown}"
    ctrl=$(timeout 12 ros2 control list_controllers 2>/dev/null \
           | awk '{printf "%s ", $1$NF}')
    info "controllers: ${ctrl:-none}"
  else
    fail "controller_manager did not answer list_hardware_components"
  fi
fi

# The contrast that makes ghosts visible instead of quietly authoritative.
echo "== what the listing CLAIMS (for contrast only) =="
listed=$(timeout 10 ros2 node list 2>/dev/null | tr '\n' ' ')
info "${listed:-<empty>}"
if [ "$failures" -gt 0 ] && [ -n "$listed" ]; then
  info "^ if a check above failed while its node appears here, that entry is a"
  info "  DDS ghost, not a running node."
fi

echo
if [ "$failures" -eq 0 ]; then
  echo "ALL CHECKS PASSED"
  exit 0
fi
echo "$failures CHECK(S) FAILED"
exit 1
