#!/usr/bin/env bash
# Faz 4 sorting GERÇEK TUTUŞ KABUL TESTİ
#
# Tam stack'i ayağa kaldırır (Gazebo + controllers + move_group + RGB-D kamera +
# perception_node + autonomous_pick_node sort_all:=true fast_sort:=false).
# N koşu boyunca sahneyi kurar, nesnelerin fiziksel olarak taşınmasını bekler
# ve GT pozlarını okuyarak doğru kutu, fırlatma ve çökme kontrolü yapar.
#
# ÖN KOŞUL: eğitilmiş YOLO modeli `src/arm_perception/models/yolo_arm.pt`.

set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
DRIVER=".claude/skills/run-6dof-arm/driver.sh"
MODEL="src/arm_perception/models/yolo_arm.pt"
LOG="/tmp/acceptance_real_grasp.log"
REPORT="reports/real_grasp_acceptance.md"

RUNS=3
WIRE_CHECK=0
OBJECTS_PER_CLASS=2
PASS_THRESHOLD=6
SETTLE_TIMEOUT=240 # Per-object timeout (nesne başına)
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

say() { echo "[real-grasp-acceptance] $*"; }

ws_source() {
  source /opt/ros/jazzy/setup.bash
  if [ ! -f install/setup.bash ]; then
    say "install/setup.bash yok — önce: colcon build --symlink-install"; exit 1
  fi
  source install/setup.bash
}

teardown() { bash "$DRIVER" down >/dev/null 2>&1 || true; }

# ---- Önkoşullar ----
ws_source

if [ ! -f "$MODEL" ]; then
  say "Eğitilmiş model YOK: $MODEL"
  if [ "$WIRE_CHECK" -eq 0 ]; then
    say "Eğitim bitince modeli koy, sonra bu script'i tekrar çalıştır."
    say "Şimdilik telleri sınamak için: bash scripts/acceptance_real_grasp.sh --wire-check"
    exit 3
  fi
fi

# ---- Tel kontrolü (modelsiz) ----
if [ "$WIRE_CHECK" -eq 1 ]; then
  say "WIRE-CHECK: config + script lint (model/sim GEREKMEZ)"
  fails=0
  
  # Check config load
  python3 -c "from arm_perception.sorting_config import load_sorting_config as L; c=L(); print('  config OK — kutular:', sorted(c.bins))" || fails=$((fails+1))
  
  # Check syntax of test script
  python3 -m py_compile src/arm_tests/integration/test_real_grasp_sorting.py || fails=$((fails+1))
  echo "  script lint OK: test_real_grasp_sorting.py"
  
  [ "$fails" -eq 0 ] && { say "WIRE-CHECK: PASS"; exit 0; } || { say "WIRE-CHECK: $fails sorun"; exit 1; }
fi

# ---- Tam stack ----
trap teardown EXIT
say "stack başlatılıyor (perception + sort_all + fast_sort:=false)... log: $LOG"
teardown
nohup ros2 launch arm_bringup perception.launch.py \
  autonomous:=true sort_all:=true fast_sort:=false use_rviz:=false \
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
  ros2 control set_controller_state joint_state_broadcaster active >/dev/null 2>&1 || true
  sleep 3
done
[ "$ready" -eq 1 ] || { say "HAZIR OLMADI — log son satırları:"; tail -n 20 "$LOG"; exit 1; }
say "stack HAZIR"

mkdir -p reports
echo "# Gerçek Tutuş Sıralama Kabul Testi Raporu" > "$REPORT"
echo "Tarih: $(date)" >> "$REPORT"
echo "" >> "$REPORT"
echo "| Run | Objects | Correct | Wrong | Fling | Crash | Verdict |" >> "$REPORT"
echo "|-----|---------|---------|-------|-------|-------|---------|" >> "$REPORT"

# ---- Koşu döngüsü ----
passes=0
for run in $(seq 1 "$RUNS"); do
  say "===== KOŞU $run / $RUNS ====="
  
  # taze sahne:
  ros2 run arm_perception demo_sorting \
    --objects-per-class "$OBJECTS_PER_CLASS" --seed "$run" 2>&1 | sed 's/^/  spawn: /' || {
      say "KOŞU $run: sahne kurulamadı — FAIL"; 
      echo "| $run | FAIL | - | - | - | - | ERROR |" >> "$REPORT"
      continue; 
    }

  # hakem: oturmayı bekle + değerlendir
  RESULT_LINE=""
  
  set +e
  python3 src/arm_tests/integration/test_real_grasp_sorting.py \
      --run-id "$run" --objects-per-class "$OBJECTS_PER_CLASS" \
      --pass-threshold "$PASS_THRESHOLD" --settle-timeout "$SETTLE_TIMEOUT" \
      --log-file "$LOG" > /tmp/acceptance_run_output.txt 2>&1
  PY_EXIT=$?
  set -e
  
  cat /tmp/acceptance_run_output.txt
  grep "^RESULT" /tmp/acceptance_run_output.txt > /tmp/acceptance_result.txt || true
  
  if [ "$PY_EXIT" -eq 0 ]; then
    passes=$((passes+1))
  fi
  
  RESULT_LINE=$(cat /tmp/acceptance_result.txt)
  
  if [ -n "$RESULT_LINE" ]; then
    # Parse: RESULT run=1 objects=6 correct=6 wrong=0 fling=0 crash=0 verdict=PASS
    correct=$(echo "$RESULT_LINE" | grep -o "correct=[0-9]*" | cut -d= -f2)
    wrong=$(echo "$RESULT_LINE" | grep -o "wrong=[0-9]*" | cut -d= -f2)
    fling=$(echo "$RESULT_LINE" | grep -o "fling=[0-9]*" | cut -d= -f2)
    crash=$(echo "$RESULT_LINE" | grep -o "crash=[0-9]*" | cut -d= -f2)
    verdict=$(echo "$RESULT_LINE" | grep -o "verdict=[A-Z]*" | cut -d= -f2)
    objects=$(echo "$RESULT_LINE" | grep -o "objects=[0-9]*" | cut -d= -f2)
    
    echo "| $run | $objects | $correct | $wrong | $fling | $crash | $verdict |" >> "$REPORT"
  else
    echo "| $run | ? | ? | ? | ? | ? | FAIL |" >> "$REPORT"
  fi

  # bir sonraki koşu için sahneyi temizle
  ros2 run arm_perception demo_sorting --reset-only --skip-ik-validation >/dev/null 2>&1 || true
done

echo "" >> "$REPORT"
echo "## Özet" >> "$REPORT"
echo "**Toplam Koşu:** $RUNS" >> "$REPORT"
echo "**Geçen Koşu:** $passes" >> "$REPORT"

if [ "$passes" -eq "$RUNS" ]; then
  echo "**Sonuç:** PASS ✅" >> "$REPORT"
else
  echo "**Sonuç:** FAIL ❌" >> "$REPORT"
fi

echo
say "================ ÖZET ================"
say "GEÇEN KOŞU: $passes / $RUNS  (eşik: her koşuda correct>=$PASS_THRESHOLD, wrong=0, fling=0, crash=0)"
if [ "$passes" -eq "$RUNS" ]; then
  say "KABUL: PASS ✅  — Rapor: $REPORT"
  exit 0
else
  say "KABUL: FAIL ❌  — log detayları için $LOG ve $REPORT dosyalarına bak"
  exit 1
fi
