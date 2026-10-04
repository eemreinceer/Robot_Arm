#!/usr/bin/env bash
# Runs inside the Nano container (Humble). Reproduces the rsp SIGINT crash.
source /opt/ros/humble/install/setup.bash >/dev/null 2>&1
RSP=$(find /opt/ros/humble/install -name robot_state_publisher -type f -perm -u+x 2>/dev/null | head -1)
echo "binary: ${RSP:-NOT FOUND}"
[ -z "$RSP" ] && exit 1
URDF=/workspace/robot_arm_test.urdf
[ -f "$URDF" ] || { echo "urdf missing"; exit 1; }

N=${1:-10}
SETTLE=${2:-3}
crashes=0; codes=""
for i in $(seq 1 "$N"); do
  "$RSP" "$URDF" >/dev/null 2>&1 &
  pid=$!
  sleep "$SETTLE"
  kill -0 "$pid" 2>/dev/null || { echo "  [$i] died before SIGINT"; continue; }
  kill -INT "$pid" 2>/dev/null
  wait "$pid" 2>/dev/null
  rc=$?
  codes="$codes $rc"
  if [ "$rc" = "139" ]; then
    crashes=$((crashes+1)); echo "  [$i] rc=$rc  <== SIGSEGV"
  else
    echo "  [$i] rc=$rc"
  fi
done
echo "---"
echo "iterations: $N   segfaults: $crashes"
echo "exit codes:$codes"
