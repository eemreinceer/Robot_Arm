# Robot Arm Console

Robot Arm'nın yerel ağda çalışan salt okunur operatör ekranıdır. Canlı kamera,
zaman damgalı YOLO kutuları, 3B nesneler, URDF robot modeli, ROS sağlık durumu
ve senkron rosbag kayıt/tekrar akışını tek sayfada birleştirir. Pi 5 sistem
telemetrisi (sıcaklık, throttle, yük, RAM, disk, uptime), kamera taşıma bilgisi,
kol görev durumu ve açık çevrim eklem tablosu da aynı canlı snapshot içindedir.

## Güvenlik sınırı

- Gateway hiçbir trajectory/action/service istemcisi veya komut publisher'ı
  oluşturmaz. `/joint_states`, gerçek servo encoder ölçümü değil, gönderilen
  açık çevrim komutun ROS tarafındaki karşılığı olarak gösterilir.
- Varsayılan HTTP bind adresi `127.0.0.1`'dir. Yerel ağ erişimi yalnız güvenilen
  robot arayüzünün IP adresi açıkça verilerek etkinleştirilir.
- Replay sırasında `/tf` ve `/tf_static` canlı ROS grafiğine tekrar basılmaz.
  Kayıtta postmortem için tutulurlar.
- Arayüzdeki kayıt düğmeleri robot hareketi başlatmaz.

## Build ve çalıştırma

```bash
cd src/arm_perception/web/robot-arm-console
npm install
npm run build

cd ../../../..
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm_interfaces arm_perception
source install/setup.bash
ros2 launch arm_perception web_console.launch.py
```

Pi 5 hedefi ROS 2 Jazzy'dir. Varsayılan adres yalnız Pi 5'in kendi tarayıcısından
erişilebilen `http://127.0.0.1:8088`'dir.

Tarayıcı adresi: `http://127.0.0.1:8088`.

### Tarihsel Nano/Humble konteyneri

Nano artık aktif hedef değildir. Geri dönüş gerektiğinde PC'deki Jazzy
`install/` ağacını Humble konteynerinde source etmeyin; Python
type-support ABI'leri farklıdır. Bu makinede doğrulanmış ayrı Humble overlay
`robot_arm_web_console` konteynerinin içindedir. Konteyner durduktan sonra yeniden
başlatmak için:

```bash
docker start robot_arm_web_console
docker exec -d \
  -e ROS_DOMAIN_ID=0 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e 'CYCLONEDDS_URI=<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp55s0"/></Interfaces></General></Domain></CycloneDDS>' \
  robot_arm_web_console bash -lc '
    source /opt/ros/humble/setup.bash
    source /tmp/robot_arm_web_console_humble_ws/install/setup.bash
    ros2 launch arm_perception web_console.launch.py \
      > /tmp/web_console.log 2>&1
  '
```

Canlılık kapısı:

```bash
curl -f http://127.0.0.1:8088/api/v1/health
docker exec robot_arm_web_console tail -n 50 /tmp/web_console.log
```

Bu konteyner reboot-sonrası otomatik başlamaz; kalıcı tek-instance başlatıcı
root orkestrasyon işidir.

Robot LAN'ındaki başka bir bilgisayardan erişilecekse gateway Humble +
`rmw_cyclonedds_cpp` ortamında, doğru Ethernet arayüzüne pinlenmiş olarak
çalıştırılır:

```bash
ros2 launch arm_perception web_console.launch.py bind_address:=127.0.0.1
```

Bu adres örnektir; PC'nin robot ağı arayüzündeki gerçek adres kullanılmalıdır.
Jetson ve PC'nin RMW/DDS ayarları mevcut `scripts/rviz_pc.sh` sözleşmesiyle aynı
olmalıdır.

## Veri sözleşmesi

- Kamera: `/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`)
- 2B kutular: `/detections_2d` (`vision_msgs/Detection2DArray`)
- 3B nesneler: `/detected_objects` (`arm_interfaces/ObjectArray`)
- Robot: `/robot_description`, `/joint_states`
- Durum: `/arm_status`
- Sistem: gateway hostundan salt-okunur Linux/Pi ölçümleri; Raspberry Pi'de
  `vcgencmd get_throttled`, diğer hostlarda desteklenmiyorsa `null`

Kamera, 2B ve 3B algılama mesajları aynı kaynak görüntü zaman damgasını
taşımalıdır. Gateway yalnız eşleşen veya en fazla 300 ms eski kutuları kareye
çizer; daha eski sonuçları aktif nesne olarak göstermez.

Kayıtlar varsayılan olarak `/var/tmp/robot-arm-console/recordings` altında tutulur.
Otomatik silme yoktur ve 5 GB boş alan kalmadığında yeni kayıt reddedilir.
