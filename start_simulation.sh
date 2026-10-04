#!/bin/bash
# =============================================================================
# 6DOF Robotic Arm — Simülasyon Başlatma Scripti
# Kullanım: ./start_simulation.sh [mod] [--no-rviz] [--headless]
#
# Modlar:
#   (VARSAYILAN)   EKSİKSİZ TAM SİM = kamera + YOLO perception + otonom sınıf-bazlı
#                  sıralama + gerçek kol hareketi + 3 kutu/6 nesneli sahneyi
#                  OTOMATİK spawn. Nesne sonucu Gazebo fiziğine bağlı kalmadan
#                  action sonrası deterministik sim sync ile kutuya yazılır.
#                  (perception.launch.py autonomous:=true sort_all:=true fast_sort:=false
#                   simulation_sync:=true simulation_post_place_sync:=true)
#                  Eğitilmiş yolo_arm.pt gerektirir.
#   --sorting      Varsayılanla aynı (açık ad; geriye dönük uyumluluk için).
#   --perception   Otonom döngü ama sahne OTOMATİK spawn EDİLMEZ (tek-yer modu).
#   --camera       perception.launch.py use_perception:=false — Gazebo + MoveIt +
#                  RGB-D kamera (YOLO perception node OLMADAN).
#   --basic        Sade mod: Gazebo + MoveIt2 + pick_place + RViz (kamera/perception YOK).
#   --robot_arm         Robot Arm küçük kol: kendi Gazebo dünyası + 5-DOF MoveIt hattı.
#                  Eski robot paketlerini veya modlarını değiştirmez.
#   --hand-eye     SİM HAND-EYE PROVASI (issue #8). Robot Arm kolu + kamera, montaj
#                  transformu YER GERÇEĞİ olarak verilir. perception / pick_place
#                  / sorting AÇILMAZ: prova pozlarını doğrudan JointTrajectory
#                  ile sürer, otonom döngü aynı controller'a yazarsa ölçüm bozulur.
#                  Montaj: --mount-xyz="x y z" --mount-rpy="r p y"
#                  (varsayılan 2026-08-12 provasının değerleri).
#                  Sonra ayrı terminalde:
#                    python3 scripts/sim_hand_eye_capture.py
#                    python3 scripts/solve_hand_eye.py --samples runs/hand_eye/sim_samples.json
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"

# Renk kodları
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()   { echo -e "${GREEN}[SIM]${NC} $1"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $1"; }
error() { echo -e "${RED}[ERR]${NC} $1"; }

# Argüman işleme
NO_RVIZ=false
HEADLESS=false
MODE="sorting"          # VARSAYILAN = eksiksiz tam sim. sorting | perception | camera | default | robot_arm | hand_eye
# Hand-eye provasının yer gerçeği montajı. Gerçek kol için bir sayı İMA ETMEZ:
# gerçekte bu transform ölçülmedi, sim'de bilinsin diye veriliyor ki çözülen X
# karşılaştırılabilsin.
MOUNT_XYZ="0.02 0.0 0.06"
MOUNT_RPY="0 0.2 0"
for arg in "$@"; do
    case $arg in
        --no-rviz)     NO_RVIZ=true ;;
        --headless)    NO_RVIZ=true; HEADLESS=true ;;
        --camera)      MODE="camera" ;;
        --perception)  MODE="perception" ;;
        --sorting)     MODE="sorting" ;;
        --basic)       MODE="default" ;;   # sade: kamera/perception yok
        --robot_arm)        MODE="robot_arm" ;;
        --hand-eye)    MODE="hand_eye" ;;
        --mount-xyz=*) MOUNT_XYZ="${arg#*=}" ;;
        --mount-rpy=*) MOUNT_RPY="${arg#*=}" ;;
    esac
done

# ROS2 ve workspace kaynak dosyaları
log "ROS2 Jazzy yükleniyor..."
source /opt/ros/jazzy/setup.bash

if [ -f "$WS_DIR/install/setup.bash" ]; then
    log "Workspace yükleniyor: $WS_DIR"
    source "$WS_DIR/install/setup.bash"
else
    error "install/setup.bash bulunamadı — önce 'colcon build' çalıştırın"
    exit 1
fi

# Display ayarı (WSL için)
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
    warn "DISPLAY ayarlanmadı, :0 kullanılıyor. Eğer GUI açılmazsa 'export DISPLAY=:0' çalıştırın."
fi

# Robot Arm hattı tamamen ayrı paket/launch kullanır. Bu erken dal sayesinde eski
# robotun arm_bringup modları ve argümanları aynen korunur.
if [ "$MODE" = "robot_arm" ] || [ "$MODE" = "hand_eye" ]; then
    if pgrep -f '[g]z sim' >/dev/null || pgrep -f '[i]gn gazebo' >/dev/null; then
        error "Başka bir Gazebo simülasyonu zaten açık. Önce onu Ctrl+C ile kapatın."
        exit 2
    fi

    REQUIRED_PACKAGES="robot_arm_description robot_arm_moveit_config"
    if [ "$MODE" = "hand_eye" ]; then
        # Prova MoveIt kullanmaz; pozları doğrudan controller'a yazar.
        REQUIRED_PACKAGES="robot_arm_description"
    fi
    for package in $REQUIRED_PACKAGES; do
        if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
            error "$package kurulu değil — önce ilgili paketleri colcon build ile derleyin."
            exit 1
        fi
    done

    ROBOT_ARM_HEADLESS="false"
    if [ "$HEADLESS" = true ]; then
        ROBOT_ARM_HEADLESS="true"
    else
        # Snap tabanlı terminallerden miras kalan GTK/GLib yolları
        # Gazebo GUI'yi /snap/core20 libpthread ile açıp GLIBC_PRIVATE hatasıyla
        # düşürüyor. Bazı terminaller SNAP değişkenini silip diğer yolları
        # bıraktığı için temizliği terminal türünden bağımsız uygula.
        warn "Gazebo GUI için olası Snap GTK/GLib ortam kalıntıları temizleniyor."
        unset SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA SNAP_EUID
        unset SNAP_INSTANCE_NAME SNAP_LAUNCHER_ARCH_TRIPLET SNAP_LIBRARY_PATH SNAP_NAME
        unset SNAP_REAL_HOME SNAP_REVISION SNAP_UID SNAP_USER_COMMON SNAP_USER_DATA SNAP_VERSION
        unset GDK_PIXBUF_MODULEDIR GDK_PIXBUF_MODULE_FILE GIO_MODULE_DIR GSETTINGS_SCHEMA_DIR
        unset GTK_EXE_PREFIX GTK_IM_MODULE_FILE GTK_PATH LOCPATH XDG_DATA_HOME
        export XDG_DATA_DIRS="/usr/local/share:/usr/share:/usr/share/ubuntu:/usr/share/gnome"
    fi

    if [ "$MODE" = "hand_eye" ]; then
        log "SİM HAND-EYE PROVASI başlatılıyor (Gazebo + kamera; MoveIt/YOLO YOK)..."
        log "Launch: robot_arm_description/gazebo.launch.py headless:=$ROBOT_ARM_HEADLESS"
        log "Kamera montajı (YER GERÇEĞİ): xyz='$MOUNT_XYZ' rpy='$MOUNT_RPY'"
        echo ""
        echo "=========================================="
        echo "  Durdurmak için: Ctrl+C"
        echo "  Bu mod otonom döngü AÇMAZ — kolu yalnız prova scripti sürer."
        echo "  Ayrı terminalde, controller'lar aktif olduktan sonra:"
        echo "    python3 scripts/sim_hand_eye_capture.py"
        echo "    python3 scripts/solve_hand_eye.py --samples runs/hand_eye/sim_samples.json"
        echo "  Yer gerçeği X ile karşılaştır: xyz='$MOUNT_XYZ' rpy='$MOUNT_RPY'"
        echo "=========================================="
        echo ""

        ros2 launch robot_arm_description gazebo.launch.py \
            use_sim_time:=true headless:="$ROBOT_ARM_HEADLESS" \
            camera_mount_xyz:="$MOUNT_XYZ" camera_mount_rpy:="$MOUNT_RPY"
        exit $?
    fi

    log "Robot Arm tam simülasyonu başlatılıyor (Gazebo + MoveIt + YOLO + sorting)..."
    log "Launch: arm_bringup/robot_arm_perception.launch.py headless:=$ROBOT_ARM_HEADLESS"
    echo ""
    echo "=========================================="
    echo "  Durdurmak için: Ctrl+C"
    echo "  Kamera kontrol: ros2 topic hz /robot_arm_cameraera"
    echo "  Controller kontrol: ros2 control list_controllers"
    echo "  Pipeline: RGB-D kamera -> YOLO -> /detected_objects -> Robot Arm pick/place"
    echo "=========================================="
    echo ""

    ros2 launch arm_bringup robot_arm_perception.launch.py \
        use_sim_time:=true headless:="$ROBOT_ARM_HEADLESS"
    exit $?
fi

# Launch dosyası + argümanları (moda göre)
LAUNCH_ARGS="use_sim_time:=true"
case $MODE in
    sorting)
        # Eksiksiz sınıf-bazlı sıralama: otonom döngü + sort_all + gerçek kol hareketi.
        # Pick/place sonucu Gazebo fizik attach/drop davranışına bırakılmaz; action
        # başarılıysa sim nesnesi deterministik set_pose ile kutuya senkronlanır.
        LAUNCH_FILE="perception.launch.py"
        LAUNCH_ARGS="$LAUNCH_ARGS autonomous:=true sort_all:=true fast_sort:=false simulation_sync:=true simulation_post_place_sync:=true pick_descent_droop_compensation_m:=0.0"
        ;;
    perception)
        LAUNCH_FILE="perception.launch.py"
        LAUNCH_ARGS="$LAUNCH_ARGS autonomous:=true"
        ;;
    camera)
        LAUNCH_FILE="perception.launch.py"
        LAUNCH_ARGS="$LAUNCH_ARGS use_perception:=false"
        ;;
    *)
        LAUNCH_FILE="pick_and_place.launch.py"
        ;;
esac
if [ "$NO_RVIZ" = true ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS use_rviz:=false"
fi

log "Simülasyon başlatılıyor (mod: $MODE)..."
log "Launch: $LAUNCH_FILE"
log "Argümanlar: $LAUNCH_ARGS"
echo ""
echo "=========================================="
echo "  Durdurmak için: Ctrl+C"
if [ "$MODE" = "sorting" ]; then
echo "  EKSİKSİZ SIRALAMA: stack hazır olunca 3 kutu + 6 nesne OTOMATİK spawn edilir."
echo "  Kol her nesne için pick/place hareketini yapar; sim nesne sonucu action sonrası kutuya senkronlanır."
echo "  Sahneyi elle yenilemek: ros2 run arm_perception demo_sorting --objects-per-class 2 --seed 1"
else
echo "  Demo çalıştır:  python3 src/arm_nodes/scripts/demo_pick_place.py"
fi
if [ "$MODE" != "default" ]; then
echo "  Kamera kontrol:  ros2 topic hz /camera/image"
echo "                   ros2 topic echo /detected_objects --once   # (perception/sorting)"
fi
echo "=========================================="
echo ""

# --- Sıralama modu: stack hazır olunca sahneyi OTOMATİK kur (arka planda) ----
if [ "$MODE" = "sorting" ]; then
    (
        set +e   # bekleme döngüleri nonzero dönebilir; alt-kabuğu erken kapatma
        # 1) controller_manager + arm_controller yüklensin
        until ros2 control list_controllers 2>/dev/null | grep -q arm_controller; do sleep 3; done
        # 2) joint_state_broadcaster aktif olsun (ağır yükte spawner zaman aşımına uğrayabilir)
        for _ in 1 2 3 4 5; do
            ros2 control list_controllers 2>/dev/null | grep joint_state_broadcaster | grep -q active && break
            ros2 control set_controller_state joint_state_broadcaster active >/dev/null 2>&1 || true
            sleep 2
        done
        # 3) perception_node yayına başlasın (launch'ta 22s gecikmeli başlar)
        until ros2 node list 2>/dev/null | grep -q perception_node; do sleep 3; done
        sleep 5
        log "Sıralama sahnesi kuruluyor (3 kutu + 6 nesne)..."
        ros2 run arm_perception demo_sorting --objects-per-class 2 --seed 1 --skip-ik-validation --object-timeout 480 \
            || warn "Sahne kurulamadı (stack tam hazır olmamış olabilir; elle tekrar deneyin)."
    ) &
    SCENE_PID=$!
    # Foreground launch bitince (Ctrl+C) arka plan sahne kurucuyu da temizle
    trap 'kill "$SCENE_PID" 2>/dev/null || true' EXIT
fi

ros2 launch arm_bringup "$LAUNCH_FILE" $LAUNCH_ARGS
