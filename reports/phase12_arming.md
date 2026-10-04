# Faz 12 — Aktivasyon Güvenliği

**Tarih:** 2026-07-18

**Kapsam:** Yalnız `src/arm_hardware/`; mock serial, ROS 2 Jazzy PC

**Fiziksel donanım:** Kullanılmadı

## Sonuç

`STM32SystemInterface` artık lifecycle aktivasyonundan sonra **disarmed** başlar.
Disarmed `write()` çağrıları başarıyla döner fakat UART `P` frame'i üretmez.
Hardware plugin içindeki `/robot_arm_hardware_safety` parametre node'u üzerinden önce
altı eklemli `reference_positions`, sonra `armed=true` verilmeden hareket komutu
geçemez.

Arming geçişinin ilk control çevrimi yalnız operatör referansını başlangıç state'i
olarak kurar ve PWM göndermez. Sonraki hedefler iki bağımsız sınırdan geçer:

1. Kanal başına `max_velocity_rad_s` (varsayılan `0.1 rad/s`).
2. Var olan `soft_start.max_delta_us_per_step` (mock kabulünde `10 us/20 ms`).

`activation_safety.calibration_complete` açıkça `true` değilse hedefler kalibrasyon
aralığına ek olarak operatör referansının `+-commissioning_range_rad` çevresinde
sınırlandırılır (varsayılan `0.3 rad`). Sınır dışı ve NaN/Inf komutlar reddedilir.

Armed durumdan disarm geçişi tam bir `S` frame'i üretir. Eski fiziksel pozun artık
geçerli olduğu varsayılmadığından referans da silinir; yeniden arm için yeni gözlem
ve yeni `reference_positions` gerekir.

## Operatör arayüzü

```bash
ros2 param set /robot_arm_hardware_safety reference_positions \
  "[q1, q2, q3, q4, q5, q6]"
ros2 param set /robot_arm_hardware_safety armed true
ros2 param set /robot_arm_hardware_safety armed false
```

`reference_positions` ROS joint sırasındadır (`joint_1..joint_6`) ve radyan
cinsindedir. Bu değer encoder ölçümü değildir; operatörün fiziksel poz tahminidir.

## Otomatik doğrulama

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm_hardware
source install/setup.bash
colcon test --packages-select arm_hardware
colcon test-result --all --verbose
```

Sonuç: build PASS; `6/6` GTest PASS; test-result toplam `7 tests`, `0 errors`,
`0 failures`, `0 skipped`. Değişen C++ dosyalarında `ament_cpplint` PASS.

Test edilen davranışlar:

- Referans olmadan arm reddi.
- Kalibrasyon tamamlanmamışken referans çevresi dışındaki hedefin reddi.
- Kanal başına rad/s sınırı.
- Disarmed `write()` sırasında sıfır mock `P` frame'i.
- Arming çevriminde sıfır `P`; sonraki hedefte pulse delta `<=10 us`.
- Disarm geçişinde yalnız bir mock `S`; eski referansla yeniden arm reddi.

## Mock ros2_control smoke

`ros2 launch arm_hardware mock_system.launch.py` ile hardware component disarmed
başladı. Referanssız arm ve `joint_6=0.4 rad` sınır dışı referans reddedildi.
Geçerli referansla üç controller ACTIVE oldu; `joint_1=0.05 rad` tek-nokta hedefi
sonrasında `/joint_states` delivered-command tahmini `0.05024 rad` gösterdi.
Disarm sonrası yeni referans verilmeden arm reddedildi. Süreç Ctrl+C ile kapandı.

İlk birleşik CLI denemesinde controller yükleme servisi geçici olarak cevap vermedi
ve süreç zorla kapatıldı. Temiz yeniden başlatmada controller'lar tek tek yüklenerek
aynı akış eksiksiz PASS oldu. Bu ilk geçici olay fiziksel kabul sayılmadı.

## Fiziksel olarak doğrulanmayanlar

- Nano/Humble build ve deployment yapılmadı.
- ESP32 UART, gerçek `P`/`S` frame'leri ve PWM pinleri ölçülmedi.
- Servo hareketi, mekanik limit, kablo güvenliği veya sıçrama davranışı denenmedi.
- Operatör referansının gerçek servo pulse/URDF sıfırıyla doğruluğu kanıtlanmadı.
- `calibration_complete=true` kullanılmadı ve önerilmedi.

İlk gerçek aktivasyon yalnız kullanıcı robot başındayken; kol mekanik destekli,
servo-ray kesici elde ve kamera/kablolar ayrıkken yapılmalıdır. ROS disarm fiziksel
acil durdurmanın yerine geçmez.
