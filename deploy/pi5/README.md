# Pi 5 systemd unit'leri — YAKALANMIŞ KOPYA

Buradaki `.service` dosyaları Pi 5'te **çalışan** unit'lerin kopyasıdır:

```
~/.config/systemd/user/robot-arm-camera.service
~/.config/systemd/user/robot-arm-dashboard.service
```

**Otorite Pi'dir, bu dizin değil.** Unit'ler Pi üzerinde kuruldu
(2026-08-21) ve deploy mekanizması da onun: izole release'ler
`${ROBOT_ARM_RUNTIME_ROOT:-/opt/robot_arm}/releases/<commit12>` altında, `current`
sembolik bağı atomik olarak çevriliyor. Buradaki kopyalar sürüm kontrolü
altında bir referans olsun diye eklendi — daha önce unit'ler yalnızca Pi'nin
diskinde duruyordu ve kart bozulursa kaybolacaklardı.

Bir unit'i değiştirdiysen kopyayı da güncelle; aksi halde bu dizin sessizce
yalan söylemeye başlar.

## İki unit ne yapar

`robot-arm-camera.service` — IMX219 kamera node'unu **`ros2 launch` olmadan**,
doğrudan çalıştırır. Gerekçe unit'in kendi yorumunda: launch bir ebeveyn süreç
ekler, o ölürse arkada izlenmeyen bir kamera çocuğu kalır. Node'un kendisi ana
süreç olunca `KillMode=control-group` bütün cgroup'u temizler.
`GST_PLUGIN_PATH` unit içinde açıkça verilir — apt'nin libcamera 0.2.0'ı yerine
`/usr/local` altındaki 0.7.x derlemesi seçilsin diye.

`robot-arm-dashboard.service` — salt okunur web konsolunu varsayılan olarak yalnız
`127.0.0.1:8088` üzerinde yayınlar. `/etc/robot_arm/robot_arm.env` içindeki
`ROBOT_ARM_BIND_ADDR` ile operatör kontrollü bir arayüze açılabilir.
`camera_config_dir` ve `measurement_registry` parametreleri
2026-08-21'de eklendi; bunlar olmadan kalibrasyon kimliği ve ölçüm tazeliği
panelleri boş kalır. Parametreler `current` üzerinden geçtiği için release
değiştikçe kendiliğinden takip eder.

## `mjpeg_bridge.py` — Pi'ye özgü varyant

`deploy/nano/mjpeg_bridge.py` ile aynı değil ve öyle olması da gerekmiyor:
Pi'deki kopya `/camera/image_raw` yerine **`/camera/pose_overlay`** konusunu
yayınlıyor. Cihazda `~/mjpeg_bridge.py` olarak duruyordu ve sürüm kontrolünde
hiç yoktu — kart bozulsa kaybolurdu. 2026-08-26'da olduğu gibi alındı.

`robot-arm-dashboard.service` de aynı gün cihazdaki gerçek hâliyle güncellendi:
unit'te fazladan `raw_image_max_fps:=30.0` parametresi vardı, repo kopyasında
yoktu. `robot-arm-camera.service` birebir aynıydı.

## Release kesme ve geri alma

Yeni release (Pi üzerinde, `~/6DOF_Robotic_Arm` istenen commit'te ve temizken):

```bash
ROBOT_ARM_RUNTIME_ROOT=${ROBOT_ARM_RUNTIME_ROOT:-/opt/robot_arm}
REL=$ROBOT_ARM_RUNTIME_ROOT/releases/$(git -C "$PWD" rev-parse HEAD | cut -c1-12)
mkdir -p "$REL/src" "$REL/scripts"
for p in arm_interfaces arm_moveit_config arm_perception arm_tests robot_arm_description; do
  cp -a ~/6DOF_Robotic_Arm/src/$p "$REL/src/"
done
cp -a ~/6DOF_Robotic_Arm/scripts/{capture_calib_images.py,solve_camera_intrinsics.py} "$REL/scripts/"
cd "$REL" && source /opt/ros/jazzy/setup.bash && colcon build --packages-up-to arm_perception
```

Swap (atomik) ve servisler:

```bash
ROOT=${ROBOT_ARM_RUNTIME_ROOT:-/opt/robot_arm}
readlink -f $ROOT/current > $ROOT/PREVIOUS_RELEASE
ln -sfn "$REL" $ROOT/current.tmp && mv -T $ROOT/current.tmp $ROOT/current
systemctl --user restart robot-arm-camera.service robot-arm-dashboard.service
```

Geri alma:

```bash
ROOT=${ROBOT_ARM_RUNTIME_ROOT:-/opt/robot_arm}
ln -sfn "$(cat $ROOT/PREVIOUS_RELEASE)" $ROOT/current.tmp && mv -T $ROOT/current.tmp $ROOT/current
systemctl --user restart robot-arm-camera.service robot-arm-dashboard.service
```

**Geri alma yolu 2026-08-21'de PROVA EDİLDİ** (`005d595` → `4c8c546` → geri):
eski release, yeni unit'in fazladan geçtiği parametrelerle sorunsuz açıldı,
`health=200`. Tanımsız parametreler eski node'u düşürmüyor — ama unit'e yeni
parametre eklerken bunu her seferinde yeniden prova et, çünkü kırılırsa
kırıldığını ancak geri almak istediğin gün öğrenirsin.

## Swap öncesi bakılacaklar

Release'i yayına almadan önce, kurduğun ağaçta:

| kontrol | beklenen |
| --- | --- |
| `grep -c first_frame_timeout_s .../csi_camera_node.py` | > 0 (watchdog var) |
| `grep ^calibration_rms_px .../imx219_640x480_pi5.yaml` | yürürlükteki kalibrasyon |
| `grep ^principal_point_policy .../imx219_640x480_pi5.yaml` | `fixed_image_center` |
| `config/measurement_registry.yaml` | var |
| `.../arm_perception/console_measurements.py` | var |

**Ve tek kamera çalıştığını doğrula.** 2026-08-21'de elle başlatılmış bir kamera
ile servisinki aynı anda koştu; ikisi de `/camera/image_raw`'a yayın yaptı ve
`CameraInfo` iki farklı kalibrasyon arasında gidip geldi. Bunu okuyan hiçbir şey
farkı göremez:

```bash
pgrep -af csi_camera_node   # tam olarak bir satir bekleniyor
```
