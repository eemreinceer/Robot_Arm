#!/usr/bin/env bash
# Robot Arm — PC tarafinda RViz ile GERCEK kolu izle (Faz 15 cozumu, 2026-07-22).
#
# Neden konteyner: PC'de native ROS Jazzy var, Nano'da Humble. Ayni DDS
# alaninda guvenilir konusmalari icin PC tarafi da Humble olmali. Faz 15
# planinin ongordugu yol buydu; calistigi 2026-07-22'de dogrulandi.
#
# Kesfin calismasi icin UC sartin hepsi gerekli:
#   1. Ayni ROS surumu    -> PC'de Humble konteyner (osrf/ros:humble-desktop)
#   2. Ayni RMW           -> Nano rmw_cyclonedds_cpp kullaniyor, PC de oyle
#                            olmali (Humble varsayilani FastDDS'tir!)
#   3. Dogru arayuz       -> CYCLONEDDS_URI ile enp55s0'a sabitlenir, yoksa
#                            Cyclone yanlis bir VPN veya container arayuzu secebilir.
#
# AG GUNCELLEMESI (2026-07-28 olcumu): 07-27'deki "arayuz pini PRATIKTE
# GEREKMIYOR" notu BAYAT. O sonuc iki ucun de varsayilan route subnet'inde
# oldugu onceki topolojiye aitti.
#
# Bugunku topoloji farkli: dogrudan kablo linki geri geldi ama PC'nin default
# route'u Wi-Fi uzerinde.
# Yani robot linki PC'de IKINCIL arayuz; pinlenmeyen her sey wifi'dan cikar.
# Ustelik 3. sart artik iki tarafta da gerekli: Nano'nun Cyclone'u pinlenmezse
# docker0'in 172.17.0.1'ini de locator olarak ilan ediyor ve PC'nin KENDI
# docker0'i ayni adreste oldugu icin eslesme tamamlanmiyor -- PC Nano'yu
# goremiyor, Nano PC'yi goruyor (asimetrik). Nano tarafi
# runtime workspace altindaki cyclonedds_eth0.xml ile eth0'a pinlendi.
#
# Olculen kanit: ham multicast ve 239.255.0.1:7400 SPDP iki yonde de temiz
# geciyordu, yani ariza ag katmaninda DEGIL locator seviyesindeydi. Pinleme
# sonrasi PC'de camera_info 29.95 Hz, image_raw 30.06 Hz.
#
# 2. sart hala gecerli: RMW esitlenmezse `ros2 node list` SESSIZCE bos doner.
# Ayrıntı: iki uçta da aynı ROS_DOMAIN_ID ve doğru DDS arayüzü kullanılmalıdır.
#
# Ayrica konteyner repoyu KENDI HOST YOLUNA mount eder: colcon --symlink-install
# ile uretilen install/ agaci mutlak host yollarina symlink'tir, baska bir yere
# mount edilirse ament paketi bulamaz ve RViz mesh'leri yukleyemez.

set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="robot_arm_viz"
IMAGE="robot_arm_viz:cyclone"       # osrf/ros:humble-desktop + ros-humble-rmw-cyclonedds-cpp
IFACE="${ROBOT_ARM_DDS_IFACE:-enp55s0}"
RVIZ_CFG="${REPO}/src/robot_arm_description/config/robot_arm_real.rviz"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[!] ${IMAGE} yok. Bir kez olusturmak icin:"
  echo "    docker run -d --name tmp --network host osrf/ros:humble-desktop sleep infinity"
  echo "    docker exec tmp apt-get update && docker exec tmp apt-get install -y ros-humble-rmw-cyclonedds-cpp"
  echo "    docker commit tmp ${IMAGE} && docker rm -f tmp"
  exit 1
fi

xhost +local:docker >/dev/null 2>&1 || true

# Onceki kosulardan kalanlari temizle. Bu iki kez isirdi: her cagri yeni bir
# rviz/hayalet takimi baslatiyor, eskisi yasamaya devam ediyor ve graf cift
# isimli dugumlerle doluyor -- bir keresinde `ros2 param set` "Node not found"
# ile dustu ve sebebi bulmak zaman aldi. Desenler koseli parantezli, yoksa
# pkill kendi kabugunu de oldurur.
if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  docker exec "${CONTAINER}" bash -lc "
    pkill -f '[r]viz2' 2>/dev/null
    pkill -f '[p]ublish_zero_joint_states' 2>/dev/null
    pkill -f '[s]tatic_transform_publisher' 2>/dev/null
    pkill -f '[r]obot_state_publisher' 2>/dev/null
    sleep 2" >/dev/null 2>&1 || true
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  docker rm -f "${CONTAINER}" >/dev/null 2>&1 || true
  docker run -d --name "${CONTAINER}" --network host \
    -e "DISPLAY=${DISPLAY:-:1}" -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "${REPO}:${REPO}" \
    "${IMAGE}" sleep infinity >/dev/null
  echo "[+] ${CONTAINER} baslatildi"
fi

ROS_ENV="
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=${REPO}/install/robot_arm_description:\$AMENT_PREFIX_PATH
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\"/></Interfaces></General></Domain></CycloneDDS>'
export DISPLAY=${DISPLAY:-:1}
"

# --ghost: URDF sifir pozunda SABIT duran saydam referans model.
# zero_offset olcumu icin sart -- canli model /joint_states ile hareket ettigi
# icin "RViz'e denk getir" derken kimildamayan bir hedef gerekiyor. Ikinci bir
# robot_state_publisher, frame_prefix=ref_ ile ayri bir TF agaci yayinlar ve
# sifir joint state alir. RViz'deki ZeroPoseGhost display'i o agaci cizer.
if [[ "${1:-}" == "--ghost" ]]; then
  # URDF'i xacro ile yeniden URETMIYORUZ: konteynerde xacro yok, ve daha
  # onemlisi hayaletin canli modelle birebir ayni tanimdan gelmesi gerekiyor.
  docker exec "${CONTAINER}" bash -lc "${ROS_ENV}
    python3 ${REPO}/scripts/dump_robot_description.py /tmp/robot_arm_ref.urdf
  " || { echo "[!] robot_description alinamadi -- Nano bringup ayakta mi?"; exit 1; }

  # frame_prefix SONUNDA SLASH ile: RViz'in RobotModel display'i TF onekini
  # prefix + "/" + link seklinde birlestirir, yani "ref_base_link" DEGIL
  # "ref/base_link" arar. Onek "ref_" birakilirsa display Error verir.
  docker exec -d "${CONTAINER}" bash -lc "${ROS_ENV}
    ros2 run robot_state_publisher robot_state_publisher /tmp/robot_arm_ref.urdf \
      --ros-args -p frame_prefix:=ref/ \
      -r /joint_states:=/ref/joint_states \
      -r /robot_description:=/ref/robot_description \
      > /tmp/ref_rsp.log 2>&1
  "

  # Hayalet agaci kendi koku (ref/world) uzerinde durur ve canli agaca hic
  # baglanmaz; RViz sabit cerceve "world"e donusturemedigi icin hicbir sey
  # cizmez. Birim static transform iki agaci birlestirir ve hayalet canli
  # modelin tam ustune oturur -- hizalamayi gozle okumak icin istedigimiz sey.
  docker exec -d "${CONTAINER}" bash -lc "${ROS_ENV}
    ros2 run tf2_ros static_transform_publisher \
      --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
      --frame-id world --child-frame-id ref/world \
      > /tmp/ref_static.log 2>&1
  "
  # `ros2 topic pub` DEGIL: o header.stamp'i 0 birakir, tf2 sifir damgali
  # donusumu kullanmaz ve hayalet hic gorunmez.
  docker exec -d "${CONTAINER}" bash -lc "${ROS_ENV}
    python3 ${REPO}/scripts/publish_zero_joint_states.py > /tmp/ref_jsp.log 2>&1
  "
  echo "[+] Sifir-poz hayaleti yayinlaniyor (ref/ TF oneki, world'e bagli)"
  echo "    NOT: kol sifirdayken hayalet canli modelin TAM ALTINDA kalir ve"
  echo "    gorunmez. Gormek icin RViz'de 'RobotModel' display'ini kapat."
  sleep 3
fi

docker exec -d "${CONTAINER}" bash -lc "${ROS_ENV}
rviz2 -d ${RVIZ_CFG} > /tmp/rviz.log 2>&1
"

echo "[+] RViz baslatildi (log: docker exec ${CONTAINER} tail -f /tmp/rviz.log)"
echo
echo "Kesfi kontrol etmek icin:"
echo "  docker exec ${CONTAINER} bash -lc 'source /opt/ros/humble/setup.bash; \\"
echo "    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; ros2 node list'"
echo "Beklenen: Nano'nun alti dugumu (/controller_manager, /robot_arm_hardware_safety, ...)"
