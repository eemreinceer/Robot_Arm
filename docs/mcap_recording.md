# MCAP kayıtları (sim)

Kayıtlar `runs/mcap/` altına yazılır. `runs/` zaten `.gitignore`'da — bag'ler
git'e girmez, üreten betikler girer. Kaydı yeniden üretmek için:

```bash
# Terminal 1 — sim
ros2 launch robot_arm_description gazebo.launch.py headless:=true

# Terminal 2 — kayıt (koreografi + throttle + bag)
./scripts/record_sim_mcap.sh
```

Çıktı: `runs/mcap/robot_arm_sim_<tarih>/robot_arm_sim_<tarih>_0.mcap`

## İçerik

| grup | topic |
| --- | --- |
| zaman/model | `/clock`, `/robot_description`, `/tf`, `/tf_static` |
| eklem durumu | `/joint_states`, `/dynamic_joint_states` |
| controller | `/robot_arm_{arm,gripper}_controller/{controller_state,joint_trajectory,transition_event}`, `/joint_state_broadcaster/transition_event`, `/controller_manager/activity` |
| kamera | `/camera/image_throttle`, `/camera/camera_info_throttle`, `/camera/depth/image_raw_throttle`, `/camera/points_throttle` |
| log | `/rosout` |

Koreografi (`scripts/sim_motion_demo.py`, ~67 s sim-zamanı): her eklem tek tek
±%80 limit taraması → pick & place benzeri 7 poz → 2 tur sinüs süpürme → home.
Gripper bunun boyunca aç/kapa döngüsü yapar.

## Neden bazı şeyler böyle

- **Açık topic listesi, `-a` değil.** Aynı ROS_DOMAIN_ID üzerinde ağdaki başka
  robotların topic'leri görünüyor (`/farmerbot_controller/*`, `/zlac8015d/*`,
  `/zed/*`, `/lora/*`); `-a` onları da bag'e sokuyordu.
- **Kamera throttle'lı.** Ham hız ~30 Hz → `/camera/image` 26.8 MB/s,
  `/camera/points` ~150 MB/s (ölçüm). Ham kayıt ~17 GB oluyordu.
  `scripts/sensor_throttle.py` 10/5/2 Hz'e düşürüyor.
- **`--compression-mode file` yok.** O mod çıktıyı `.mcap.zstd` yapıyor ve
  dosya artık geçerli MCAP değil ("invalid magic bytes", ölçüldü). Bunun yerine
  `config/mcap_writer.yaml` ile MCAP'in kendi chunk-zstd'si: 442 MB → 9 MB.
- **Kayıt SIGTERM ile durdurulur.** Arka plandaki `ros2 bag record` bu Jazzy'de
  SIGINT'e cevap vermiyor (üç INT sonrası hâlâ yazıyordu); SIGTERM cache'i
  boşaltıp `metadata.yaml`'ı tamamlıyor.

## Okuma / oynatma

```bash
ros2 bag info runs/mcap/<kayıt>
ros2 bag play runs/mcap/<kayıt> --clock
```

`.mcap` dosyası doğrudan Foxglove ile de açılır (dosyayı sürükle-bırak).

## Bilinen sınırlar

- Transport katmanında ~60 mesaj düşüyor (80 bin mesajın ~%0.07'si), çoğunlukla
  ~1.6 kHz `/clock`.
- Headless sim gerçek zamandan hızlı koşuyor: 85 s sim-zamanı ~28 s duvar
  saatinde kaydedildi. Kamera throttle'ı duvar saatiyle çalıştığı için
  sim-zamanına göre efektif kare hızı daha düşüktür.
- `camera_mount_joint` transformu hâlâ ölçülmedi (bkz. `robot_arm_camera.xacro`),
  yani bag'deki kamera→`base_link` TF'i gerçeği temsil etmez.
