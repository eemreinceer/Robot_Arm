#!/usr/bin/env bash
# Gazebo sim'inden geniş kapsamlı bir MCAP kaydı üretir.
#
#   Terminal 1: ros2 launch robot_arm_description gazebo.launch.py headless:=true
#   Terminal 2: ./scripts/record_sim_mcap.sh [çıktı_dizini]
#
# Kayıt AÇIK bir topic listesiyle yapılır (`-a` DEĞİL): aynı ROS_DOMAIN_ID
# üzerinde ağdaki başka robotların (farmerbot, zlac8015d, zed ...) topic'leri
# görünüyor ve `-a` onları da bag'e sokuyor. Liste robot_arm kolunun kendi grafiğidir.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${1:-$REPO_ROOT/runs/mcap/robot_arm_sim_$(date +%Y%m%d_%H%M%S)}"

# ROS setup betikleri tanımsız değişken okur → `set -u` geçici olarak kapatılır.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"
set -u

TOPICS=(
  # --- zaman ve model ---
  /clock
  /robot_description
  /tf
  /tf_static
  # --- eklem durumu ---
  /joint_states
  /dynamic_joint_states
  # --- controller'lar ---
  /robot_arm_controller/controller_state
  /robot_arm_controller/joint_trajectory
  /robot_arm_controller/transition_event
  /robot_arm_gripper_controller/controller_state
  /robot_arm_gripper_controller/joint_trajectory
  /robot_arm_gripper_controller/transition_event
  /joint_state_broadcaster/transition_event
  /controller_manager/activity
  # --- kamera (sim RGB + derinlik + nokta bulutu, throttle'lı) ---
  # Ham hız ~30 Hz → ~215 MB/s (ölçüldü). sensor_throttle.py 10/5/2 Hz'e
  # düşürüyor; ham kayıt ~17 GB oluyordu.
  /camera/image_throttle
  /camera/camera_info_throttle
  /camera/depth/image_raw_throttle
  /camera/points_throttle
  # --- log ---
  /rosout
)

# Kayıt, koreografi bitene kadar sürer (+ önden/arkadan pay).
MOTION_S="$(python3 "$REPO_ROOT/scripts/sim_motion_demo.py" --print-duration-only)"

echo "[record] çıktı  : $OUT_DIR"
echo "[record] hareket: ${MOTION_S}s"
mkdir -p "$(dirname "$OUT_DIR")"

python3 "$REPO_ROOT/scripts/sensor_throttle.py" &
THROTTLE_PID=$!
trap 'kill -INT "$THROTTLE_PID" 2>/dev/null || true' EXIT
sleep 3  # throttle abonelikleri + discovery

# `--compression-mode file` KULLANILMAZ: çıktı `.mcap.zstd` oluyor ve artık
# geçerli bir MCAP dosyası değil (Foxglove/`mcap` CLI "invalid magic bytes"
# veriyor, ölçüldü). MCAP zaten chunk seviyesinde zstd sıkıştırıyor.
# --max-cache-size: /clock ~1.6 kHz + 3 controller state topic'i 100 MB
# varsayılanını taşırıp transport katmanında mesaj düşürüyordu.
ros2 bag record \
  --storage mcap \
  --storage-config-file "$REPO_ROOT/config/mcap_writer.yaml" \
  --output "$OUT_DIR" \
  --max-cache-size 536870912 \
  --topics "${TOPICS[@]}" &
BAG_PID=$!
trap 'kill -TERM "$BAG_PID" "$THROTTLE_PID" 2>/dev/null || true' EXIT

sleep 3  # kayıt aboneliklerinin bağlanması
python3 "$REPO_ROOT/scripts/sim_motion_demo.py" || true

sleep 2
# SIGINT bu Jazzy'de arka plandaki `ros2 bag record`'u durdurmuyor (ölçüldü:
# üç INT sonrası hâlâ yazıyordu). SIGTERM temiz kapatıyor: cache boşaltılıyor,
# zstd sıkıştırma ve metadata.yaml tamamlanıyor.
kill -TERM "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true
kill -TERM "$THROTTLE_PID" 2>/dev/null || true
trap - EXIT
wait "$THROTTLE_PID" 2>/dev/null || true

echo
ros2 bag info "$OUT_DIR"
