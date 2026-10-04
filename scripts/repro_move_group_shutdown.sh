#!/usr/bin/env bash
# Issue #14 — move_group kapanış kusurunun İZOLE yeniden üretimi.
#
# NEDEN ÇIPLAK BINARY
#   Daha önceki denemeler `ros2 launch` üzerinden koştu ve kusuru launch
#   sırasından/timeout'undan ayıramadı. Bu betik `move_group` ELF'ini doğrudan
#   çalıştırır: launch yok, ros2_control yok, spawner yok, başka node yok.
#   Burada üreyen bir kusur tanım gereği repo'nun launch dosyalarına ait
#   olamaz.
#
# NEDEN DEBUGINFOD ÖNEMLİ
#   `move_group` kapanışta SIGSEGV veriyor. backward_ros'un sinyal işleyicisi
#   stack'i sembolleştirmek için libdw üzerinden debuginfod'a HTTPS isteği
#   yapar. `DEBUGINFOD_URLS` doluysa bu istek kapanışı SINIRSIZ bloklar ve
#   çökme, "SIGINT'te asıldı → SIGTERM/-15" gibi GÖRÜNÜR. Değişken boşken
#   aynı çökme ~1.6 s'de -11 olarak görünür. Bu betik ikisini de ölçebilir.
#
# KULLANIM
#   scripts/repro_move_group_shutdown.sh            # debuginfod KAPALI (kanonik)
#   scripts/repro_move_group_shutdown.sh --debuginfod   # A/B: açık, asılmayı gösterir
#
# ÇIKIŞ KODU
#   0  beklenen kusur görüldü (kapalı modda -11, açık modda sınırsız asılma)
#   1  beklenmeyen davranış — politikayı yeniden gözden geçir
#
# GÜVENLİ: hiçbir robot, UART, servo rayı, PWM, flash veya simülasyon yok.
set +u
set -o pipefail

WITH_DEBUGINFOD=0
[ "${1:-}" = "--debuginfod" ] && WITH_DEBUGINFOD=1

WS_SETUP="$(cd "$(dirname "$0")/.." && pwd)/install/setup.bash"
source /opt/ros/jazzy/setup.bash
[ -f "$WS_SETUP" ] && source "$WS_SETUP"

# Kendi ROS domain'i: makinede koşan başka bir stack'i etkilemesin.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"

if [ "$WITH_DEBUGINFOD" -eq 1 ]; then
  export DEBUGINFOD_URLS="${DEBUGINFOD_URLS:-https://debuginfod.ubuntu.com}"
  DEADLINE=45
else
  export DEBUGINFOD_URLS=""
  DEADLINE=30
fi

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
PARAMS="$WORK/move_group_params.yaml"
LOG="$WORK/move_group.log"

python3 - "$PARAMS" <<'PY'
import subprocess, os, sys, yaml
from ament_index_python.packages import get_package_share_directory

desc = get_package_share_directory("robot_arm_description")
mc = get_package_share_directory("robot_arm_moveit_config")
urdf = subprocess.run(
    ["xacro", os.path.join(desc, "urdf", "robot_arm_real.urdf.xacro"),
     "serial_device:=/dev/ttyUSB0", "baud_rate:=115200", "mock_serial:=true"],
    capture_output=True, text=True, check=True).stdout

def raw(n):
    return open(os.path.join(mc, "config", n)).read()

def y(n):
    return yaml.safe_load(open(os.path.join(mc, "config", n)))

params = {
    "robot_description": urdf,
    "robot_description_semantic": raw("robot_arm.srdf"),
    "robot_description_kinematics": y("kinematics.yaml"),
    "robot_description_planning": y("joint_limits_real.yaml"),
    "use_sim_time": False,
}
params.update(y("ompl_planning.yaml"))
params.update(y("moveit_controllers.yaml"))
yaml.safe_dump({"/**": {"ros__parameters": params}}, open(sys.argv[1], "w"))
PY
[ $? -eq 0 ] || { echo "params üretilemedi"; exit 1; }

BIN="$(ros2 pkg prefix moveit_ros_move_group)/lib/moveit_ros_move_group/move_group"
echo "== binary : $BIN"
echo "== sha256 : $(sha256sum "$BIN" | cut -d' ' -f1)"
echo "== sürüm  : $(ros2 pkg xml -t version moveit_ros_move_group)"
echo "== DEBUGINFOD_URLS='${DEBUGINFOD_URLS}'"

# setsid: kendi proses grubunda dursun, SIGINT'i yalnız hedefe verelim.
setsid "$BIN" --ros-args --params-file "$PARAMS" > "$LOG" 2>&1 &
PID=$!

for i in $(seq 1 60); do
  grep -q "You can start planning now" "$LOG" && break
  kill -0 "$PID" 2>/dev/null || { echo "FAIL: proses erken öldü"; tail -20 "$LOG"; exit 1; }
  sleep 1
done
grep -q "You can start planning now" "$LOG" || { echo "FAIL: move_group hazır olmadı"; exit 1; }
sleep 2

START=$(date +%s.%N)
kill -INT "$PID"
for i in $(seq 1 $((DEADLINE * 10))); do
  kill -0 "$PID" 2>/dev/null || break
  sleep 0.1
done

if kill -0 "$PID" 2>/dev/null; then
  ELAPSED=$(echo "$(date +%s.%N)-$START" | bc)
  kill -9 -"$PID" 2>/dev/null
  echo "== SONUÇ: ${ELAPSED}s boyunca SIGINT'e yanıt vermedi (asıldı)"
  if [ "$WITH_DEBUGINFOD" -eq 1 ]; then
    echo "BEKLENEN: debuginfod açıkken çökme sembolleştirmede sınırsız bloklanır."
    exit 0
  fi
  echo "BEKLENMEYEN: debuginfod kapalıyken asılma olmamalıydı."
  exit 1
fi

wait "$PID" 2>/dev/null; RC=$?
ELAPSED=$(echo "$(date +%s.%N)-$START" | bc)
echo "== SONUÇ: rc=$RC, süre=${ELAPSED}s"
grep -q "Deleting MoveItCpp" "$LOG" && echo "== 'Deleting MoveItCpp' görüldü"
tail -3 "$LOG"

if [ "$WITH_DEBUGINFOD" -eq 1 ]; then
  echo "BEKLENMEYEN: debuginfod açıkken asılma bekleniyordu, rc=$RC alındı."
  exit 1
fi

# 139 = 128 + 11 (SIGSEGV)
if [ "$RC" -eq 139 ]; then
  echo "BEKLENEN: MOVEIT_JAZZY_2_12_4_TEARDOWN_SIGSEGV (rc=139 / sinyal 11)"
  exit 0
fi
echo "BEKLENMEYEN rc=$RC — docs/m3_shutdown_policy.md yeniden gözden geçirilmeli."
exit 1
