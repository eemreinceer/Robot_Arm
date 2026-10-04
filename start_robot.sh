#!/usr/bin/env bash
# Canonical Robot Arm real-hardware entry point.
# Starts only the fail-closed, DISARMED control plane by default.

set -Eeuo pipefail

ROBOT_HOST="${ROBOT_ARM_HOST:-}"
ROBOT_USER="${ROBOT_ARM_USER:-robot_arm}"
HW_CONTAINER="${ROBOT_ARM_HW_CONTAINER:-robot_arm_hw}"
SERIAL_DEVICE="${ROBOT_ARM_SERIAL_DEVICE:-/dev/ttyTHS1}"
EXPECTED_FW="${ROBOT_ARM_EXPECTED_FW:-1.2.2-esp32}"
EXPECTED_CALIB="${ROBOT_ARM_EXPECTED_CALIB:-caf4354aa8ae}"
REMOTE_LOG="/tmp/robot_arm_hardware_bringup.log"
MODE="start"
REFERENCE=""

usage() {
  cat <<'EOF'
Usage:
  ./start_robot.sh
      Start and verify the hardware stack DISARMED. Does not enable PWM,
      arm controllers, or move the robot.

  ./start_robot.sh --arm '[q1,q2,q3,q4,q5,q6]'
      With the servo rail still OFF and the arm physically supported, set the
      observed ROS-joint reference, arm the host gate, spawn both motion
      controllers, and verify them. The operator powers the servo rail manually
      only after this command succeeds.

  ./start_robot.sh --status
      Show the current ROS hardware/controller/safety state.

  ./start_robot.sh --stop
      After the servo rail is physically OFF, disarm and stop the ROS stack.

Environment overrides:
  ROBOT_ARM_HOST, ROBOT_ARM_USER, ROBOT_ARM_HW_CONTAINER, ROBOT_ARM_SERIAL_DEVICE,
  ROBOT_ARM_EXPECTED_FW, ROBOT_ARM_EXPECTED_CALIB
EOF
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

require_confirmation() {
  local prompt="$1"
  local answer
  if [[ "${ROBOT_ARM_CONFIRMED_SAFE:-}" == "YES" ]]; then
    return
  fi
  read -r -p "$prompt Type EVET: " answer
  [[ "$answer" == "EVET" ]] || die "Safety confirmation was not received."
}

remote() {
  [[ -n "$ROBOT_HOST" ]] ||
    die "ROBOT_ARM_HOST is required (hostname or operator-supplied address)."
  ssh \
    -o BatchMode=yes \
    -o ConnectTimeout=7 \
    -o ServerAliveInterval=5 \
    -o ServerAliveCountMax=2 \
    "${ROBOT_USER}@${ROBOT_HOST}" "$@"
}

docker_bash() {
  local command="$1"
  local quoted_command quoted_container
  printf -v quoted_command '%q' "$command"
  printf -v quoted_container '%q' "$HW_CONTAINER"
  remote "docker exec $quoted_container bash -c $quoted_command"
}

ros_in_container() {
  local command="$1"
  docker_bash "source /opt/ros/humble/install/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
$command"
}

validate_reference() {
  REFERENCE="$(python3 - "$REFERENCE" <<'PY'
import math
import sys

text = sys.argv[1].strip()
if not (text.startswith("[") and text.endswith("]")):
    raise SystemExit("reference must use '[q1,q2,q3,q4,q5,q6]' syntax")
try:
    values = [float(item.strip()) for item in text[1:-1].split(",")]
except ValueError as exc:
    raise SystemExit(f"reference contains a non-number: {exc}")
if len(values) != 6 or not all(math.isfinite(value) for value in values):
    raise SystemExit("reference must contain exactly six finite radians")
print("[" + ",".join(repr(value) for value in values) + "]")
PY
)"
}

assert_connection() {
  command -v ssh >/dev/null || die "ssh is not installed."
  remote true || die "Jetson SSH is unreachable: ${ROBOT_USER}@${ROBOT_HOST}"
}

ensure_container() {
  local state
  state="$(remote docker inspect -f '{{.State.Status}}' "$HW_CONTAINER" 2>/dev/null)" ||
    die "Container does not exist: $HW_CONTAINER"
  if [[ "$state" != "running" ]]; then
    remote docker start "$HW_CONTAINER" >/dev/null
  fi
  remote docker exec "$HW_CONTAINER" test -c "$SERIAL_DEVICE" ||
    die "$SERIAL_DEVICE is not a character device inside $HW_CONTAINER."
  remote docker exec "$HW_CONTAINER" test -f /workspace/install/setup.bash ||
    die "/workspace/install/setup.bash is missing inside $HW_CONTAINER."
}

stack_pids() {
  docker_bash \
    "pgrep -af '[r]os2 launch robot_arm_description hardware_bringup.launch.py|[r]os2_control_node|[c]ontroller_manager' || true"
}

uart_fail_closed_preflight() {
  local output version ping state angles
  output="$(remote docker exec -i "$HW_CONTAINER" bash -s -- "$SERIAL_DEVICE" <<'REMOTE'
set -Eeuo pipefail
device="$1"
stty -F "$device" 115200 cs8 -cstopb -parenb -ixon -ixoff -crtscts raw -echo
exec 3<>"$device"

# S is the locked protocol's safe stop: disable PWM before inspection.
printf 'S\n' >&3
IFS= read -r -t 2 stop_reply <&3 || { echo "STOP_TIMEOUT"; exit 20; }
[[ "$stop_reply" == "OK" ]] || { echo "STOP_REPLY=$stop_reply"; exit 21; }

printf 'V?\n' >&3
IFS= read -r -t 2 version <&3 || { echo "VERSION_TIMEOUT"; exit 22; }
printf 'PING\n' >&3
IFS= read -r -t 2 ping <&3 || { echo "PING_TIMEOUT"; exit 23; }
printf 'STATUS\n' >&3
IFS= read -r -t 2 state <&3 || { echo "STATUS_TIMEOUT"; exit 24; }
IFS= read -r -t 2 angles <&3 || { echo "ANGLES_TIMEOUT"; exit 25; }
printf '%s\n%s\n%s\n%s\n' "$version" "$ping" "$state" "$angles"
REMOTE
)" || die "UART fail-closed preflight failed: ${output:-no response}"

  mapfile -t lines <<<"$output"
  [[ "${#lines[@]}" -eq 4 ]] || die "UART preflight returned an unexpected line count."
  version="${lines[0]}"
  ping="${lines[1]}"
  state="${lines[2]}"
  angles="${lines[3]}"

  [[ "$version" == "V1,$EXPECTED_FW" ]] ||
    die "Firmware mismatch: got '$version', expected 'V1,$EXPECTED_FW'."
  [[ "$ping" == *"FW=$EXPECTED_FW"* ]] ||
    die "PING firmware mismatch: $ping"
  [[ "$ping" == *"FRAME=ROS_JOINT_DEG"* ]] ||
    die "Firmware frame mismatch: $ping"
  [[ "$ping" == *"CALIB=$EXPECTED_CALIB"* ]] ||
    die "Firmware calibration mismatch: $ping"
  [[ "$state" == STATE,DISARMED,* && "$state" == *"MOVING=0"* &&
     "$state" == *"QUEUE=0"* && "$state" == *"PWM=0"* ]] ||
    die "Firmware is not fail-closed: $state"
  [[ "$state" == *"CALIB=$EXPECTED_CALIB"* ]] ||
    die "STATUS calibration mismatch: $state"
  [[ "$angles" == ANGLES,* ]] || die "Malformed ANGLES response: $angles"

  printf 'UART PASS: %s; DISARMED/MOVING=0/QUEUE=0/PWM=0; CALIB=%s\n' \
    "$EXPECTED_FW" "$EXPECTED_CALIB"
}

wait_for_control_plane() {
  local deadline=$((SECONDS + 45))
  while (( SECONDS < deadline )); do
    if ros_in_container \
      "ros2 service list | grep -Fx '/controller_manager/list_controllers' >/dev/null &&
       ros2 node list | grep -Fx '/robot_arm_hardware_safety' >/dev/null"; then
      return
    fi
    sleep 1
  done
  remote docker exec "$HW_CONTAINER" tail -n 80 "$REMOTE_LOG" || true
  die "Hardware control plane did not become ready within 45 seconds."
}

verify_disarmed() {
  local armed reference controllers
  armed="$(ros_in_container "ros2 param get /robot_arm_hardware_safety armed")"
  reference="$(ros_in_container \
    "ros2 param get /robot_arm_hardware_safety reference_positions")"
  controllers="$(ros_in_container \
    "ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'")"
  [[ "$armed" == *"False"* ]] || die "Host safety gate is not DISARMED: $armed"
  [[ "$reference" == *"array('d')"* || "$reference" == *"[]"* ]] ||
    die "Startup reference is unexpectedly non-empty: $reference"
  [[ "$controllers" == *"joint_state_broadcaster"* &&
     "$controllers" != *"robot_arm_controller"* &&
     "$controllers" != *"robot_arm_gripper_controller"* ]] ||
    die "Unexpected controller set during DISARMED startup."
}

start_stack() {
  require_confirmation \
    "Servo rail must be OFF, arm supported, work envelope clear, cutoff reachable."
  assert_connection
  ensure_container
  local existing
  existing="$(stack_pids)"
  [[ -z "$existing" ]] || die "A hardware stack is already running:
$existing"
  uart_fail_closed_preflight
  local launch_command quoted_launch quoted_container
  launch_command="source /opt/ros/humble/install/setup.bash
source /workspace/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec ros2 launch robot_arm_description hardware_bringup.launch.py \
  serial_device:=$SERIAL_DEVICE baud_rate:=115200 mock_serial:=false \
  spawn_motion_controllers:=false >$REMOTE_LOG 2>&1"
  printf -v quoted_launch '%q' "$launch_command"
  printf -v quoted_container '%q' "$HW_CONTAINER"
  remote "docker exec -d $quoted_container bash -c $quoted_launch"
  wait_for_control_plane
  verify_disarmed
  printf '%s\n' \
    "START PASS: hardware control plane is active and DISARMED." \
    "No PWM/motion controller was enabled. Servo rail must remain OFF." \
    "Next: ./start_robot.sh --arm '[q1,q2,q3,q4,q5,q6]'"
}

arm_stack() {
  local reference_result arm_result armed controllers hardware
  local active_count claimed_count
  validate_reference
  require_confirmation \
    "Servo rail must still be OFF; arm must be supported and physically at the supplied reference."
  assert_connection
  ensure_container
  [[ -n "$(stack_pids)" ]] || die "Hardware stack is not running. Run ./start_robot.sh first."
  verify_disarmed

  reference_result="$(ros_in_container \
    "ros2 param set /robot_arm_hardware_safety reference_positions '$REFERENCE'")"
  [[ "$reference_result" == *"Set parameter successful"* ]] ||
    die "Operator reference was rejected; no controller was spawned: $reference_result"

  arm_result="$(ros_in_container \
    "ros2 param set /robot_arm_hardware_safety armed true")"
  [[ "$arm_result" == *"Set parameter successful"* ]] ||
    die "Host arming was rejected; no controller was spawned: $arm_result"

  armed="$(ros_in_container "ros2 param get /robot_arm_hardware_safety armed")"
  [[ "$armed" == *"True"* ]] || {
    ros_in_container \
      "ros2 param set /robot_arm_hardware_safety armed false" >/dev/null || true
    die "Host safety gate did not report armed=True; no controller was spawned."
  }

  ros_in_container \
    "ros2 run controller_manager spawner robot_arm_controller \
      --controller-manager /controller_manager --controller-manager-timeout 60"
  ros_in_container \
    "ros2 run controller_manager spawner robot_arm_gripper_controller \
      --controller-manager /controller_manager --controller-manager-timeout 60"

  armed="$(ros_in_container "ros2 param get /robot_arm_hardware_safety armed")"
  controllers="$(ros_in_container \
    "ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'")"
  hardware="$(ros_in_container \
    "ros2 service call /controller_manager/list_hardware_components controller_manager_msgs/srv/ListHardwareComponents '{}'")"
  active_count="$(
    grep -Eo "state(: |=)'?active'?" <<<"$controllers" | wc -l
  )"
  claimed_count="$(
    grep -Eo "is_claimed(: |=)(true|True)" <<<"$hardware" | wc -l
  )"
  [[ "$armed" == *"True"* ]] || die "Host arming failed: $armed"
  [[ "$controllers" == *"robot_arm_controller"* &&
     "$controllers" == *"robot_arm_gripper_controller"* &&
     "$active_count" -ge 3 ]] ||
    die "All three controllers are not active."
  [[ "$hardware" == *"robot_arm_real_system"* && "$hardware" == *"active"* ]] ||
    die "Hardware component is not active."
  [[ "$claimed_count" -eq 6 ]] ||
    die "Expected six claimed position command interfaces, got $claimed_count."

  printf '%s\n' \
    "ARM PASS: reference accepted and both motion controllers are active." \
    "The script did not power the servos or send a trajectory." \
    "Operator: keep the cutoff in hand, then physically enable the servo rail." \
    "Observe for 10 seconds without sending a motion command."
}

show_status() {
  assert_connection
  ensure_container
  printf '%s\n' "Processes:"
  stack_pids
  printf '%s\n' "Safety:"
  ros_in_container \
    "ros2 param get /robot_arm_hardware_safety armed
ros2 param get /robot_arm_hardware_safety reference_positions"
  printf '%s\n' "Controllers:"
  ros_in_container \
    "ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'"
  printf '%s\n' "Hardware:"
  ros_in_container \
    "ros2 service call /controller_manager/list_hardware_components controller_manager_msgs/srv/ListHardwareComponents '{}'"
}

stop_stack() {
  require_confirmation "Servo rail must be physically OFF and the arm supported."
  assert_connection
  ensure_container
  if [[ -z "$(stack_pids)" ]]; then
    printf 'STOP PASS: no hardware stack is running.\n'
    return
  fi
  ros_in_container \
    "ros2 param set /robot_arm_hardware_safety armed false" >/dev/null || true
  docker_bash \
    "pkill -INT -f '[r]os2 launch robot_arm_description hardware_bringup.launch.py' || true"
  local deadline=$((SECONDS + 15))
  while (( SECONDS < deadline )); do
    [[ -z "$(stack_pids)" ]] && {
      printf 'STOP PASS: stack stopped after host disarm. Servo rail remains operator-controlled.\n'
      return
    }
    sleep 1
  done
  die "Stack did not stop cleanly; inspect $REMOTE_LOG on the Jetson."
}

while (($#)); do
  case "$1" in
    --arm)
      [[ $# -ge 2 ]] || die "--arm requires a six-value reference."
      MODE="arm"
      REFERENCE="$2"
      shift 2
      ;;
    --status)
      MODE="status"
      shift
      ;;
    --stop)
      MODE="stop"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown argument: $1"
      ;;
  esac
done

case "$MODE" in
  start) start_stack ;;
  arm) arm_stack ;;
  status) show_status ;;
  stop) stop_stack ;;
esac
