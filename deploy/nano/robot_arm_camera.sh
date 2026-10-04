#!/usr/bin/env bash
#
# Canonical definition of the Robot Arm camera container.
#
# Why this file exists
# --------------------
# Until 2026-08-05 the `robot_arm_camera` container was created by hand on the Nano and
# existed nowhere else. Two defects that had already been FIXED in this repo
# came back, because a fix that lands on `main` never reaches a container whose
# command was typed once and then survived reboots:
#
#   * The CycloneDDS eth0 pinning (found 2026-07-28) was simply not in the
#     container. At boot, eth0 had no address yet, so Cyclone bound the VPN
#     and the PC saw ZERO publishers for /camera/image_raw.
#   * The container ran `ros2 run ... -p flip_method:=2` with no
#     `camera_info_file`, so csi_camera_node refused to publish CameraInfo --
#     correctly, it will not claim an uncalibrated camera is calibrated. The
#     launch file always binds it. Recorded 2026-07-27, open for ten days.
#
# So the container command lives here now, in version control, and every
# recreate ends in the existing acceptance gate (scripts/check_alive.sh).
#
# This script runs on the PC and drives the Nano over SSH, like start_robot.sh.
#
# NOT covered here: the Nano workspace overlay (/workspace/install) is still
# built on the Nano and unversioned. The preflight refuses to build a container
# around a missing overlay, but it cannot prove the overlay matches this repo.

set -Eeuo pipefail

ROBOT_HOST="${ROBOT_ARM_HOST:-}"
ROBOT_USER="${ROBOT_ARM_USER:-robot_arm}"
CONTAINER="${ROBOT_ARM_CAMERA_CONTAINER:-robot_arm_camera}"
IMAGE="${ROBOT_ARM_CAMERA_IMAGE:-dustynv/ros:humble-ros-base-l4t-r32.7.1}"
WS_HOST="${ROBOT_ARM_WS_HOST:-/opt/robot_arm/robot_ws}"
DDS_IFACE="${ROBOT_ARM_DDS_IFACE:-eth0}"
ROS_DOMAIN="${ROBOT_ARM_ROS_DOMAIN_ID:-0}"
CAMERA_LAUNCH="${ROBOT_ARM_CAMERA_LAUNCH:-imx219_camera.launch.py}"
BRIDGE_PORT="${ROBOT_ARM_BRIDGE_PORT:-8080}"
# The launch file needs a moment before the bridge subscribes; measured startup
# to first frame is ~9 s, so this is that plus margin.
BRIDGE_DELAY="${ROBOT_ARM_BRIDGE_DELAY:-12}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# Files this repo owns and ships to the Nano on every recreate. The container
# runs these copies, so the repo is the source of truth for both.
DDS_CONFIG_SRC="$REPO_ROOT/deploy/nano/cyclonedds_eth0.xml"
DDS_CONFIG_DST="$WS_HOST/cyclonedds_eth0.xml"
BRIDGE_SRC="$REPO_ROOT/deploy/nano/mjpeg_bridge.py"
BRIDGE_DST="$WS_HOST/mjpeg_bridge.py"

MODE="status"

usage() {
  cat <<'EOF'
Usage:
  deploy/nano/robot_arm_camera.sh --status
      Report what is actually running: container state, the DDS interface the
      camera node advertises, and the MJPEG bridge. Read-only.

  deploy/nano/robot_arm_camera.sh --recreate
      Rebuild the container from this definition. The existing container is
      RENAMED (never deleted) and its restart policy disabled, so a reboot
      cannot bring two cameras up fighting over Argus and the bridge port.
      Ships deploy/nano/cyclonedds_eth0.xml to the Nano first.

  deploy/nano/robot_arm_camera.sh --verify
      Run the acceptance gate against the running container. This is
      scripts/check_alive.sh --camera, executed on the PC, because a
      wrong-interface DDS binding is only visible from the other machine.

Environment overrides:
  ROBOT_ARM_HOST, ROBOT_ARM_USER, ROBOT_ARM_CAMERA_CONTAINER, ROBOT_ARM_CAMERA_IMAGE,
  ROBOT_ARM_WS_HOST, ROBOT_ARM_DDS_IFACE, ROBOT_ARM_ROS_DOMAIN_ID, ROBOT_ARM_CAMERA_LAUNCH,
  ROBOT_ARM_BRIDGE_PORT, ROBOT_ARM_BRIDGE_DELAY
EOF
}

case "${1:---status}" in
  --status)   MODE="status" ;;
  --recreate) MODE="recreate" ;;
  --verify)   MODE="verify" ;;
  --help|-h)  usage; exit 0 ;;
  *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
esac

if [[ -z "$ROBOT_HOST" ]]; then
  echo "ROBOT_ARM_HOST is required (hostname or operator-supplied address)." >&2
  exit 2
fi

SSH=(ssh -o ConnectTimeout=8 -o BatchMode=yes "${ROBOT_USER}@${ROBOT_HOST}")

pass() { printf "  \033[32mPASS\033[0m  %s\n" "$1"; }
fail() { printf "  \033[31mFAIL\033[0m  %s\n" "$1"; }
info() { printf "        %s\n" "$1"; }

remote() { "${SSH[@]}" "$@"; }

# ---------------------------------------------------------------------------
# status
# ---------------------------------------------------------------------------
show_status() {
  echo "== container =="
  local state
  state=$(remote "docker ps -a --filter name=^/${CONTAINER}\$ --format '{{.Status}}'" || true)
  if [ -z "$state" ]; then
    fail "$CONTAINER does not exist"
  else
    info "$CONTAINER: $state"
  fi

  echo "== container command and DDS env (what is ACTUALLY running) =="
  remote "docker inspect ${CONTAINER} --format 'CMD={{json .Config.Cmd}}' 2>/dev/null" || true
  local uri
  uri=$(remote "docker exec ${CONTAINER} printenv CYCLONEDDS_URI 2>/dev/null" || true)
  if [ -z "$uri" ]; then
    fail "CYCLONEDDS_URI is NOT set -- the camera node will bind whatever
        interface came up first at boot. This is the 2026-08-05 defect."
  else
    pass "CYCLONEDDS_URI=$uri"
  fi

  echo "== MJPEG bridge =="
  if remote "curl -s -m 5 -o /dev/null -w '%{http_code}' http://127.0.0.1:${BRIDGE_PORT}/" \
       | grep -q '^200$'; then
    pass "bridge answers on :${BRIDGE_PORT}"
    info "a 200 here does NOT prove the camera node is alive -- the bridge is"
    info "PID 1 and outlives it. Use --verify for that."
  else
    fail "bridge does not answer on :${BRIDGE_PORT}"
  fi
}

# ---------------------------------------------------------------------------
# recreate
# ---------------------------------------------------------------------------
preflight() {
  echo "== preflight (fail-closed) =="
  local failures=0
  local check

  # The boot race that caused the 2026-08-05 outage: if eth0 has no address,
  # Cyclone cannot bind it and the pinning silently does nothing.
  if check=$(remote "ip -4 -br addr show ${DDS_IFACE} 2>/dev/null" || true); [ -n "$check" ] \
     && echo "$check" | grep -qE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+'; then
    pass "${DDS_IFACE} has an address: $(echo "$check" | awk '{print $3}')"
  else
    fail "${DDS_IFACE} has NO IPv4 address. Pinning DDS to it now would produce
        a container that cannot be discovered. Wait for the link and retry."
    failures=$((failures+1))
  fi

  if remote "docker image inspect ${IMAGE} >/dev/null 2>&1"; then
    pass "image present: ${IMAGE}"
  else
    fail "image missing: ${IMAGE}"
    failures=$((failures+1))
  fi

  if remote "test -f ${WS_HOST}/install/setup.bash"; then
    pass "workspace overlay present"
  else
    fail "${WS_HOST}/install/setup.bash missing -- nothing to source"
    failures=$((failures+1))
  fi

  # Shipped from the repo, so the check is on this side.
  local f
  for f in "$DDS_CONFIG_SRC" "$BRIDGE_SRC"; do
    if [ -f "$f" ]; then
      pass "repo source present: deploy/nano/$(basename "$f")"
    else
      fail "missing repo source: $f"
      failures=$((failures+1))
    fi
  done

  # Without the packaged intrinsics the node starts fine and simply never
  # publishes CameraInfo. That silence is exactly what went unnoticed for ten
  # days, so it is a hard gate here rather than a runtime surprise.
  if remote "test -f ${WS_HOST}/install/arm_perception/share/arm_perception/config/imx219_640x480.yaml"; then
    pass "packaged intrinsics present"
  else
    fail "imx219_640x480.yaml not installed -- the launch file would bind
        camera_info_file to a missing path and CameraInfo would stay silent"
    failures=$((failures+1))
  fi

  if [ "$failures" -gt 0 ]; then
    echo
    echo "$failures preflight check(s) failed; refusing to recreate."
    exit 1
  fi
}

recreate() {
  preflight

  echo "== ship repo-owned files to the Nano =="
  # Keep one dated copy of whatever is already there. These have been edited in
  # place on the Nano before, and an unversioned edit that nobody captured is
  # exactly what this directory exists to end -- but it must not be destroyed
  # on the way out either.
  local stamp
  stamp=$(date +%Y%m%d_%H%M%S)
  local src dst
  for src in "$DDS_CONFIG_SRC:$DDS_CONFIG_DST" "$BRIDGE_SRC:$BRIDGE_DST"; do
    dst="${src#*:}"; src="${src%%:*}"
    if remote "test -f ${dst}" && ! remote "sha256sum ${dst}" \
         | grep -q "$(sha256sum "$src" | awk '{print $1}')"; then
      remote "cp -p ${dst} ${dst}.pre_deploy_${stamp}"
      info "Nano copy differed; preserved as $(basename "$dst").pre_deploy_${stamp}"
    fi
    scp -q -o BatchMode=yes "$src" "${ROBOT_USER}@${ROBOT_HOST}:${dst}"
    pass "$(basename "$src") -> ${dst}"
  done

  local backup
  backup="${CONTAINER}_backup_${stamp}"

  if remote "docker ps -a --format '{{.Names}}' | grep -qx ${CONTAINER}"; then
    echo "== preserve the existing container =="
    remote "docker stop ${CONTAINER} >/dev/null 2>&1 || true"
    remote "docker rename ${CONTAINER} ${backup}"
    # Without this a reboot starts the backup too: two Argus consumers and two
    # servers on the bridge port.
    remote "docker update --restart=no ${backup} >/dev/null"
    pass "renamed -> ${backup} (restart disabled, NOT deleted)"
  else
    info "no existing ${CONTAINER} to preserve"
  fi

  echo "== create =="
  remote "docker run -d --name ${CONTAINER} \
    --network host \
    --privileged \
    --runtime nvidia \
    --restart unless-stopped \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v ${WS_HOST}:/workspace \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN} \
    -e ROS_LOCALHOST_ONLY=0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///workspace/$(basename "$DDS_CONFIG_DST") \
    ${IMAGE} \
    bash -lc 'source /workspace/install/setup.bash; \
      ros2 launch arm_perception ${CAMERA_LAUNCH} > /workspace/csi_camera.log 2>&1 & \
      sleep ${BRIDGE_DELAY}; \
      exec python3 /workspace/mjpeg_bridge.py'" >/dev/null
  pass "created ${CONTAINER}"

  echo "== wait for the bridge =="
  if remote "for i in \$(seq 1 20); do \
               curl -s -m 3 -o /dev/null http://127.0.0.1:${BRIDGE_PORT}/ && exit 0; \
               sleep 3; \
             done; exit 1"; then
    pass "bridge up on :${BRIDGE_PORT}"
  else
    fail "bridge never came up; see ${WS_HOST}/csi_camera.log"
    exit 1
  fi

  echo
  echo "Now run the acceptance gate:  deploy/nano/robot_arm_camera.sh --verify"
}

# ---------------------------------------------------------------------------
# verify
# ---------------------------------------------------------------------------
verify() {
  local gate="$REPO_ROOT/scripts/check_alive.sh"
  if [ ! -x "$gate" ]; then
    echo "missing acceptance gate: $gate" >&2
    exit 2
  fi

  # The PC talks to the Nano over the wired link, NOT over the VPN
  # address used for SSH -- so this interface is deliberately independent of
  # ROBOT_ARM_HOST.
  local pc_iface="${ROBOT_ARM_PC_DDS_IFACE:-enp55s0}"

  if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/jazzy/setup.bash ]; then
      # The ROS setup scripts reference unbound variables; `set -u` kills them.
      # scripts/check_alive.sh carries the same note.
      set +u
      # shellcheck disable=SC1091
      source /opt/ros/jazzy/setup.bash
      set -u
    else
      echo "no ROS environment; source one first" >&2
      exit 2
    fi
  fi
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$ROS_DOMAIN}"
  export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
  export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${pc_iface}\"/></Interfaces></General></Domain></CycloneDDS>}"

  echo "== acceptance gate (scripts/check_alive.sh --camera) =="
  echo "Run from the PC on purpose: a camera node bound to the wrong interface"
  echo "looks perfectly healthy on the Nano and invisible from here."
  info "PC interface ${pc_iface}, domain ${ROS_DOMAIN_ID}, ${ROS_DISTRO}"
  info "'node list' comes back empty across Jazzy<->Humble even when the data"
  info "flows -- measured. Trust the rates, not the listing."
  echo
  exec "$gate" --camera
}

case "$MODE" in
  status)   show_status ;;
  recreate) recreate ;;
  verify)   verify ;;
esac
