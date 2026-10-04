#!/usr/bin/env bash
# Faz 4 sorting KABUL TESTİ — GO sonrası tek komut.
#
# Tam stack'i ayağa kaldırır (Gazebo + controllers + move_group + RGB-D kamera +
# perception_node + autonomous_pick_node sort_all:=true), N koşu boyunca sahneyi
# kurar (demo_sorting), ayırmanın oturmasını bekler ve her nesnenin doğru kutuda
# bitip bitmediğini hakem (acceptance_check) ile ölçer.
#
# GEÇTİ ölçütü (codex Faz 4B spec): tüm koşular PASS — her koşuda
# correct >= eşik (vars. 6) ve wrong == 0.
#
# ÖN KOŞUL: eğitilmiş YOLO modeli `src/arm_perception/models/yolo_arm.pt`.
# Model yokken --wire-check ile telleri (config/IK/launch hazırlığı) sınayabilirsin.
#
# Kullanım:
#   bash scripts/acceptance_sorting.sh                 # 5 koşu, tam test
#   bash scripts/acceptance_sorting.sh --runs 3
#   bash scripts/acceptance_sorting.sh --wire-check    # modelsiz tel/önkoşul kontrolü
#
# NOT: set -u KULLANMA — ROS2 setup.bash tanımsız değişkene dokunur (skill gotcha).
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
DRIVER=".claude/skills/run-6dof-arm/driver.sh"
WORLD="${WORLD:-pick_and_place_world}"
MODEL="src/arm_perception/models/yolo_arm.pt"
LOG="/tmp/acceptance_sorting.log"

RUNS=5
WIRE_CHECK=0
OBJECTS_PER_CLASS=2
PASS_THRESHOLD=6
SETTLE_TIMEOUT=240
READY_TIMEOUT=180
while [ $# -gt 0 ]; do
  case "$1" in
    --runs) RUNS="$2"; shift 2 ;;
    --wire-check) WIRE_CHECK=1; shift ;;
    --objects-per-class) OBJECTS_PER_CLASS="$2"; shift 2 ;;
    --pass-threshold) PASS_THRESHOLD="$2"; shift 2 ;;
    --settle-timeout) SETTLE_TIMEOUT="$2"; shift 2 ;;
    *) echo "bilinmeyen arg: $1"; exit 2 ;;
  esac
done

say() { echo "[acceptance] $*"; }

ws_source() {
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [ ! -f install/setup.bash ]; then
    say "install/setup.bash yok — önce: colcon build --symlink-install"; exit 1
  fi
  # shellcheck disable=SC1091
  source install/setup.bash
}

teardown() { bash "$DRIVER" down >/dev/null 2>&1 || true; }

# ---- Önkoşullar ----
ws_source

if [ ! -f "$MODEL" ]; then
  say "Eğitilmiş model YOK: $MODEL"
  if [ "$WIRE_CHECK" -eq 0 ]; then
    say "Eğitim bitince modeli koy, sonra bu script'i tekrar çalıştır."
    say "Şimdilik telleri sınamak için: bash scripts/acceptance_sorting.sh --wire-check"
    exit 3
  fi
fi

# ---- Tel kontrolü (modelsiz) ----
if [ "$WIRE_CHECK" -eq 1 ]; then
  say "WIRE-CHECK: config + IK + entry-point hazırlığı (model GEREKMEZ)"
  fails=0
  python3 -c "from arm_perception.sorting_config import load_sorting_config as L; c=L(); print('  config OK — kutular:', sorted(c.bins))" || fails=$((fails+1))
  for ep in demo_sorting autonomous_pick_node perception_node; do
    if ros2 pkg executables arm_perception 2>/dev/null | grep -q "$ep"; then
      echo "  entry-point OK: $ep"
    else
      echo "  EKSİK entry-point: $ep"; fails=$((fails+1))
    fi
  done
  if ros2 pkg executables arm_perception 2>/dev/null | grep -q acceptance_check; then
    echo "  entry-point OK: acceptance_check"
  else
    echo "  NOT: acceptance_check entry-point yok — 'python3 -m arm_perception.acceptance_check' ile de çağrılır"
  fi
  [ "$fails" -eq 0 ] && { say "WIRE-CHECK: PASS"; exit 0; } || { say "WIRE-CHECK: $fails sorun"; exit 1; }
fi

# ---- Tam stack ----
trap teardown EXIT
say "stack başlatılıyor (perception + sort_all)... log: $LOG"
teardown
nohup ros2 launch arm_bringup perception.launch.py \
  autonomous:=true sort_all:=true use_rviz:=false \
  >"$LOG" 2>&1 &
say "launch PID $!"

# hazırlık: controller_manager + move_group + perception_node + /detected_objects
say "stack hazırlığı bekleniyor (max ${READY_TIMEOUT}s)..."
deadline=$(( $(date +%s) + READY_TIMEOUT ))
ready=0
while [ "$(date +%s)" -lt "$deadline" ]; do
  if ros2 node list 2>/dev/null | grep -q perception_node \
     && ros2 control list_controllers 2>/dev/null | grep -q "joint_state_broadcaster.*active" \
     && ros2 topic list 2>/dev/null | grep -q "/detected_objects"; then
    ready=1; break
  fi
  # skill gotcha: ağır yükte broadcaster inactive kalabilir → zorla aktive et
  ros2 control set_controller_state joint_state_broadcaster active >/dev/null 2>&1 || true
  sleep 3
done
[ "$ready" -eq 1 ] || { say "HAZIR OLMADI — log son satırları:"; tail -n 20 "$LOG"; exit 1; }
say "stack HAZIR"

# ---- Koşu döngüsü ----
passes=0
for run in $(seq 1 "$RUNS"); do
  say "===== KOŞU $run / $RUNS ====="
  # taze sahne: kutular + sınıf başına nesneler (seed koşuyla değişir)
  ros2 run arm_perception demo_sorting \
    --objects-per-class "$OBJECTS_PER_CLASS" --seed "$run" 2>&1 | sed 's/^/  spawn: /' || {
      say "KOŞU $run: sahne kurulamadı (IK/spawn hata) — FAIL"; continue; }

  # hakem: oturmayı bekle + doğru-kutu say
  if python3 -m arm_perception.acceptance_check \
      --run-id "$run" --objects-per-class "$OBJECTS_PER_CLASS" \
      --pass-threshold "$PASS_THRESHOLD" --settle-timeout "$SETTLE_TIMEOUT" \
      --world "$WORLD"; then
    passes=$((passes+1))
  fi
  # bir sonraki koşu için sahneyi temizle
  ros2 run arm_perception demo_sorting --reset-only --skip-ik-validation >/dev/null 2>&1 || true
done

echo
say "================ ÖZET ================"
say "GEÇEN KOŞU: $passes / $RUNS  (eşik: her koşuda correct>=$PASS_THRESHOLD, wrong=0)"
if [ "$passes" -eq "$RUNS" ]; then
  say "KABUL: PASS ✅  — sorting GO'ya hazır"
  exit 0
else
  say "KABUL: FAIL ❌  — $LOG ve yukarıdaki koşu detaylarına bak"
  exit 1
fi
