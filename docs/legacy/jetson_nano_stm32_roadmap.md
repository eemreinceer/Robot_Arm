# Legacy: 6DOF Arm — Jetson Nano + STM32 Integration Report

> Historical record, retained for engineering context. Jetson Nano and the
> legacy 6-DOF packages are not the active runtime. For the current Raspberry
> Pi 5 / Robot Arm architecture, see [`../../ARCHITECTURE.md`](../../ARCHITECTURE.md)
> and [`../supported_versions.md`](../supported_versions.md).

**Tarih:** 2 Temmuz 2026
**Proje:** Robot Arm (legacy 6-DOF phase)
**Donanım:** Jetson Nano (P3450, JetPack 4.6) + STM32F103C6T6A + 6x Servo

---

## 1. BUGÜNE KADAR YAPILANLAR (Jetson Nano Ortam Kurulumu)

### 1.1 Temel Sistem
- Jetson Nano headless çalışacak şekilde yapılandırıldı (`multi-user.target`, SSH erişimi)
- Ethernet üzerinden laptop ile doğrudan bağlantı kuruldu, laptop internet paylaşımı (bridge) sağlıyor
- `pipe` kullanıcısı `docker` grubuna eklendi (izin sorunu çözüldü)
- 8GB swap alanı zaten mevcuttu (JetPack varsayılanı)

### 1.2 Docker Ortamı
- `dustynv/ros:humble-ros-base-l4t-r32.7.1` image'ı kullanıldı (JetPack 4.6/L4T r32.7.1 ile uyumlu)
- Kalıcı container: `humble_dev` (`--name humble_dev`, `--rm` KULLANILMADI — durdurulup tekrar başlatılabilir)
- Volume mount: `~/robot_ws` (host) → `/workspace` (container)
- TensorRT 8.2.1.8 doğrulandı, GPU erişimi çalışıyor (`--runtime nvidia`)
- **Container'a tekrar bağlanma:** `docker start -ai humble_dev`
- **tmux session:** `build` adında — build'in SSH kopmalarına karşı korunması için kullanıldı. `tmux attach -t build` ile bağlanılır. **NOT: Nano yeniden başlatıldığında (reboot/shutdown) tmux session KAYBOLUR, container ve içindeki veriler kaybolmaz — sadece `tmux new -s build` ile session'ı yeniden açıp `docker start -ai humble_dev` çalıştırmak yeterli.**

### 1.3 MoveIt2'nin Sıfırdan Derlenmesi (En Büyük Emek)

**Neden gerekliydi:** JetPack 4.6 = Ubuntu 18.04 (bionic), GCC 7.5. MoveIt2/ROS2 Humble ise Ubuntu 22.04 (jammy) hedefliyor. Bu ~4 yıllık fark yüzünden hiçbir MoveIt2 bağımlılığı apt'te güncel/uyumlu halde bulunamadı; tamamı kaynaktan derlendi.

**Kaynaktan derlenen/eklenen paketler (sırasıyla karşılaşılan bağımlılık zinciri):**

| Paket | Yöntem | Not |
|---|---|---|
| control_msgs | git clone (humble) | rosdep'te yoktu |
| object_recognition_msgs | git clone (wg-perception org, ros2 branch) | yanlış org'da arandı, düzeltildi |
| octomap_msgs | git clone (ros2 branch) | |
| angles | git clone (ros2 branch) | |
| eigen_stl_containers | git clone (ros/eigen_stl_containers, ros2 branch) | moveit2.repos'ta eksik, bilinen bug |
| FCL | **kaynaktan cmake build**, v0.6.1→sonra master | apt'teki 0.5.0 CMake config sağlamıyordu; ARM'de SSE flag hatası (`-DFCL_USE_X64_SSE=OFF` ile çözüldü) |
| libccd-dev | apt | FCL bağımlılığı |
| generate_parameter_library | git clone (PickNikRobotics) | |
| tl::expected (libexpected) | **kaynaktan cmake build** (TartanLlama/expected) | apt'te yok |
| rsl | git clone (PickNikRobotics/RSL) | |
| tcb_span | git clone (**PickNikRobotics/cpp_polyfills**, DOĞRU repo) | tcbrindle/span'de install target yok, cpp_polyfills kullanılmalı |
| fmt (v10.1.1) | **kaynaktan cmake build**, `-DCMAKE_POSITION_INDEPENDENT_CODE=ON` ŞART | apt'teki 4.0.0 çok eski (`fmt/ranges.h` yok); PIC olmadan .so linkinde hata verir |
| geometric_shapes | git clone (moveit org, ros2 branch) | |
| random_numbers | git clone (ros-planning, ros2 branch) | CMakeLists + ConfigExtras.cmake içindeki `find_package(Boost CONFIG ...)` → `find_package(Boost ...)` olarak patch edildi (bionic Boost 1.65 CONFIG modu desteklemiyor) |
| resource_retriever | git clone (ros org, humble branch) | |
| assimp (v5.2.5) | **kaynaktan cmake build** | apt'teki 4.1.0 `assimp::assimp` CMake target export etmiyor (bilinen bionic bug) |
| srdfdom | git clone (moveit org, ros2 branch) | urdfdom_py için `pip3 install urdf-parser-py` |
| liburdfdom-dev | apt | |
| ruckig | git clone **v0.15.3 tag** (ana branch C++20 istiyor, GCC 7.5 desteklemiyor; v0.15.3 hâlâ C++17) | |
| warehouse_ros | git clone (moveit org, ros2 branch) | `tf2_ros/buffer.hpp` → `.h` patch edildi |
| OMPL (v1.6.0) | **kaynaktan cmake build** | apt'teki 1.2.1 çok eski (`ConstrainedStateSpace.h` yok) |
| g++-8 | apt | GCC 7.5'in C++17 `std::optional`/`std::variant` desteği eksik; `rclcpp::Client` bazı paketlerde derlenemiyor |

**GCC 7.5 → g++-8 ile derlenen paketler** (rclcpp::Client uyumsuzluğu nedeniyle):
- moveit_ros_planning
- moveit_kinematics
- moveit_planners_ompl
- moveit_ros_planning_interface
- arm_nodes (projenin kendi paketi)

**`<filesystem>` → `<experimental/filesystem>` + `-lstdc++fs` patch'i uygulanan dosyalar** (GCC7'de tam filesystem desteği yok):
- `moveit_core/utils/src/robot_model_test_utils.cpp`
- `moveit_ros/planning/rdf_loader/src/rdf_loader.cpp`
- (moveit_kinematics ve moveit_planners_ompl'daki aynı sorunlar g++-8 kullanılarak dolaylı çözüldü)

**Build komutu (genel):**
```bash
cd /workspace
colcon build --packages-up-to arm_nodes arm_perception --parallel-workers 1 --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

**Belirli bir paketi g++-8 ile derlemek için:**
```bash
colcon build --packages-select <paket_adi> --cmake-args -DCMAKE_CXX_COMPILER=g++-8 -DBUILD_TESTING=OFF
```

### 1.4 Projenin Kendi Kodundaki Düzeltmeler
`arm_nodes` paketinde (laptop/x86_64 + farklı MoveIt2 sürümü ile yazılmıştı):
- `pick_place_node.hpp`: `moveit/move_group_interface/move_group_interface.hpp` → `.h`
- `pick_place_node.hpp`: `moveit/planning_scene_interface/planning_scene_interface.hpp` → `.h`
- `pick_place_node.cpp`: `moveit/robot_state/robot_state.hpp`, `robot_trajectory.hpp`, `time_optimal_trajectory_generation.hpp`, `moveit_error_code.hpp` → hepsi `.h`
- `pick_place_node.cpp`: `computeCartesianPath()` çağrılarına eksik `jump_threshold` (double, `0.0`) parametresi eklendi — bu Nano'daki MoveIt2 sürümünün API imzası laptoptakinden farklı (muhtemelen sürüm farkı)

### 1.5 SONUÇ: BAŞARILI BUILD
```
Summary: 38 packages finished
4 packages had stderr output (sadece deprecation uyarıları, hata değil)
```
`arm_nodes` ve `arm_perception` paketleri Jetson Nano üzerinde (ARM64, bionic, Humble) tam olarak derlendi ve çalıştırılabilir durumda.

**Node testi:** `ros2 run arm_nodes pick_place_node` çalıştırıldığında action server sorunsuz ayağa kalktı (`Pick/place action server ready on /pick_and_place`) — binary/link sağlam. `robot_description` parametresi verilmediği için (tek başına, launch dosyasız çalıştırıldığından) bekleneni buldu, bu bir hata değil.

---

## 2. PROJENİN MEVCUT MİMARİSİ (Keşfedilenler)

### 2.1 start_simulation.sh
Bu script **SADECE laptop/Gazebo simülasyonu içindir**:
- `source /opt/ros/jazzy/setup.bash` (Nano'da Jazzy değil, Humble var)
- Gazebo + RViz + DISPLAY gerektiriyor (Nano'da anlamsız/çalışmaz)
- Modlar: `sorting` (varsayılan, tam otonom), `perception`, `camera`, `basic`

### 2.2 Launch Dosyaları (`arm_bringup/launch/`)
- `perception.launch.py` — YOLO perception + otonom sıralama (Gazebo bağımlı)
- `pick_and_place.launch.py` — **incelendi**, tam yapı: Gazebo → (7s sonra) move_group → (20s sonra) pick_place_node → RViz
- `sim_only.launch.py` — muhtemelen sadece Gazebo

**`pick_and_place.launch.py`'den öğrenilen kritik bilgi — pick_place_node'un ihtiyaç duyduğu parametreler:**
```python
moveit_params = {
    "robot_description": <xacro'dan derlenen URDF>,
    "robot_description_semantic": <arm.srdf içeriği>,
    "robot_description_kinematics": <kinematics.yaml>,
    "robot_description_planning": <joint_limits.yaml>,  # TOTG için ŞART, yoksa "No acceleration limit" hatası
    "use_sim_time": ...,
}
```

### 2.3 ros2_control Yapısı
**Controller config** (`arm_gazebo/config/ros2_controllers.yaml`):
```yaml
controller_manager:
  update_rate: 100
  joint_state_broadcaster: joint_state_broadcaster/JointStateBroadcaster
  arm_controller: joint_trajectory_controller/JointTrajectoryController
  gripper_controller: parallel_gripper_action_controller/GripperActionController

arm_controller:
  joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
  command_interfaces: [position]
  state_interfaces: [position, velocity]

gripper_controller:
  joint: gripper_joint1
  max_effort: 100.0
```

**Hardware plugin tanımı** (`arm_description/urdf/arm.urdf.xacro` içinde `<ros2_control>` bloğu):
```xml
<ros2_control name="6dof_arm_system" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>  <!-- BUNU DEĞİŞTİRECEĞİZ -->
  </hardware>
  <joint name="joint_1">
    <command_interface name="position"><param name="min">...</param><param name="max">...</param></command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  <!-- joint_2 ... joint_6 aynı yapıda -->
</ros2_control>
```

**ÖNEMLİ:** Şu an projede gerçek donanım (STM32) için **hiçbir hardware_interface plugin'i veya STM32 firmware'i yok**. İkisi de sıfırdan yazılacak.

---

## 3. HEDEF MİMARİ

```
┌─────────────┐         ┌──────────────┐         ┌─────────────────┐
│   Laptop    │  Wi-Fi/  │  Jetson Nano │  USB/   │  STM32F103C6T6A  │
│  (opsiyonel │◄────────►│  (Humble,    │◄───────►│  (firmware,      │
│  Gazebo/    │  Ethernet│  planlama +  │  UART   │  6x servo PWM)   │
│  RViz)      │          │  MoveIt2 +   │         │                  │
│             │          │  execution)  │         │                  │
└─────────────┘         └──────────────┘         └─────────────────┘
```

- **Laptop:** Sadece görselleştirme/debug (RViz), gerekmedikçe kullanılmayacak
- **Nano:** move_group (MoveIt2 planlama), pick_place_node, arm_perception (YOLO), yeni yazılacak STM32 hardware_interface plugin'i
- **STM32:** Gerçek zamanlı servo PWM üretimi, Nano'dan gelen komutları uygulama, (opsiyonel) pozisyon geri bildirimi

---

## 4. YAPILMASI GEREKENLER (Yol Haritası)

### AŞAMA A — STM32 Firmware (Sıfırdan)
1. **Donanım bağlantı planı:** 6 servo hangi STM32 timer/PWM pinlerine bağlanacak (STM32F103C6T6A'da PWM kapasiteli timer sayısı sınırlı, 6 kanal için pin planı çıkarılmalı)
2. **UART haberleşme protokolü tasarımı** — öneri: basit, insan-okunabilir text protokolü ile başlamak (debug kolaylığı):
   - Nano→STM32: `"P1500,1500,1500,1500,1500,1500\n"` (6 servo için pulse width, µs) ya da açı bazlı `"A90,45,180,...\n"`
   - STM32→Nano (opsiyonel, sadece "ACK" veya gerçek pozisyon varsa): `"OK\n"` ya da encoder/pot okuması
3. **HAL ile PWM çıkışı kurulumu** (STM32CubeMX ile timer/PWM konfigürasyonu)
4. **UART interrupt/DMA ile komut alma** (bloklamayan, gerçek zamanlı okuma)
5. **Basit bir test:** Nano'dan `screen`/`minicom` ile elle komut gönderip tek bir servonun döndüğünü doğrulama (ROS2 katmanına geçmeden önce)

### AŞAMA B — Nano Tarafı: ROS2 Hardware Interface Plugin
1. Yeni paket oluştur: `arm_hardware` (örnek isim)
2. `hardware_interface::SystemInterface` sınıfından türeyen C++ plugin yaz:
   - `on_init()` — seri port aç, parametreleri oku
   - `export_state_interfaces()` / `export_command_interfaces()` — joint_1..6 position/velocity/effort
   - `read()` — STM32'den pozisyon oku (varsa), yoksa son komut edilen değeri "gerçek" gibi döndür (open-loop)
   - `write()` — MoveIt2'den gelen hedef pozisyonları STM32'ye seri port üzerinden yolla
3. `pluginlib` ile export et (`arm_hardware_plugins.xml`)
4. `package.xml`'e `hardware_interface`, `pluginlib` bağımlılıklarını ekle

### AŞAMA C — URDF/Launch Güncellemeleri
1. `arm.urdf.xacro` içindeki `<plugin>gz_ros2_control/GazeboSimSystem</plugin>` satırını `<plugin>arm_hardware/STM32SystemInterface</plugin>` (gerçek isimle) olarak değiştir — **muhtemelen ayrı bir xacro dosyası** (`arm_real.urdf.xacro` gibi) olarak tutmak, Gazebo simülasyonunu bozmamak için daha güvenli
2. Yeni launch dosyası oluştur: `real_hardware.launch.py`
   - Gazebo YOK
   - `robot_state_publisher` (gerçek donanım URDF'i ile)
   - `controller_manager` (ros2_control_node, gerçek hardware plugin ile)
   - `move_group` (mevcut `move_group.launch.py` yeniden kullanılabilir)
   - `pick_place_node` (mevcut, parametreler `pick_and_place.launch.py`'den kopyalanacak)
   - RViz **opsiyonel** (`use_rviz` argümanıyla, varsayılan `false` — laptoptan bağlanmak istenirse `true`)

### AŞAMA D — Entegrasyon Testi
1. STM32 bağlı değilken: `real_hardware.launch.py` çalıştırılıp `move_group`'un çökmeden ayağa kalktığını doğrula (hardware plugin seri portu bulamazsa hata verecek — bu normal, plugin'in en azından yüklendiğini/pluginlib tarafından tanındığını görmek yeterli)
2. STM32 bağlıyken: Tek bir joint için basit bir `ros2 topic pub` ile `arm_controller` topic'ine hedef pozisyon gönderip servonun döndüğünü doğrula
3. Tam pick/place action'ı test et

---

## 5. AÇIK SORULAR / NETLEŞTİRİLMESİ GEREKENLER

1. STM32 tarafında **kaç UART portu** kullanılabilir/planlanıyor — tek port üzerinden 6 servo mu, yoksa farklı bir yapı mı?
2. Servolar **pozisyon geri bildirimi** verebiliyor mu (potansiyometreli/analog servo, ya da encoder'lı) yoksa **tamamen open-loop** (RC hobi servo gibi, sadece PWM ile komut, geri bildirim yok) mu çalışılacak? Bu, hardware interface'in `state_interface` davranışını doğrudan belirler.
3. STM32'nin Nano'ya bağlantı şekli: **USB (ST-Link/USB-UART çevirici üzerinden CDC)** mi, yoksa **ayrı bir USB-TTL adaptörle GPIO UART pinleri** mi kullanılacak?
4. Gripper (`gripper_joint1`) için de STM32 üzerinden mi kontrol planlanıyor, yoksa ayrı bir mekanizma mı var?

---

## 6. HIZLI REFERANS — Nano'ya Bağlanma ve Kaldığın Yerden Devam

```bash
# Laptop'tan Nano'ya bağlan
ssh "${ROBOT_ARM_USER:-robot_arm}@${ROBOT_ARM_HOST:?set ROBOT_ARM_HOST}"

# tmux session'ı bul/aç
tmux attach -t build          # varsa
tmux new -s build             # yoksa (Nano reboot olduysa)

# Container'a gir
docker start -ai humble_dev

# Workspace'i source et
cd /workspace
source /opt/ros/humble/install/setup.bash
source /workspace/install/setup.bash

# Build almak istersen (BUILD_TESTING kapalı, tek worker — Nano için güvenli)
colcon build --packages-up-to arm_nodes arm_perception --parallel-workers 1 --symlink-install --cmake-args -DBUILD_TESTING=OFF
```
