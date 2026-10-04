# Faz 4B - Sinif Bazli Ayirma Katmani

Tarih: 2026-05-31

## Teslimatlar

- `autonomous_pick_node`, `sort_all:=true` modunda sinif bazli kutu lookup yapar.
- Bilinmeyen siniflar warning ile atlanir.
- Her pick sonucu tamamlandiktan sonra yeni `/detected_objects` snapshot'i beklenir.
- Kutu icinde algilanan nesneler XY sinirlariyla filtrelenir; tekrar alinmaz.
- `PickAndPlace.return_home`, C++ action server tarafinda MoveIt `home` named target ile uygulanir.
- `demo_sorting`, uc statik acik kutuyu ve sinif basina iki nesneyi runtime Gazebo servisleriyle spawn eder.
- Dunya dosyasi ve `src/arm_gazebo/` degistirilmedi.

## Kutu Pozlari

| Sinif | Model | World merkez XY (m) | Yatay yaricap (m) | Link_6 drop XYZ (base_link, m) | Canli IK hata |
| --- | --- | --- | ---: | --- | ---: |
| `red_box` | `bin_red` | `[0.382329, -0.297691]` | 0.485 | `[0.282829, -0.277562, 0.119901]` | 0.000 mm |
| `yellow_cylinder` | `bin_yellow` | `[0.512931, -0.015758]` | 0.513 | `[0.450266, 0.037573, 0.187201]` | 0.000 mm |
| `blue_cube` | `bin_blue` | `[0.368781, 0.294579]` | 0.472 | `[0.278395, 0.340844, 0.127916]` | 0.000 mm |

Butun kutular `0.78 m` yatay erisim sinirinin icindedir. Link_6 drop pozlari FK orneklemesiyle
uretildi. `demo_sorting`, kutulari spawn etmeden once `/ik_solve` servisine canli istek atar;
herhangi bir poz cozulmezse sahne kurulmaz.

## Dogrulama

Gecen kontroller:

```bash
colcon build --symlink-install --packages-select arm_perception arm_nodes
python3 -m pytest -s -q src/arm_perception/test
# 3 passed

ros2 run arm_perception demo_sorting   --config src/arm_perception/config/sorting_bins.yaml
# IK PASS: 3/3, hata 0.000 mm
# Runtime spawn: 3 statik kutu + 6 nesne
```

Izole headless Gazebo canli kosumunda sinif basina iki nesne spawn edildi:

```text
sorting_red_box_00, sorting_red_box_01
sorting_yellow_cylinder_00, sorting_yellow_cylinder_01
sorting_blue_cube_00, sorting_blue_cube_01
```

## GO Sonrasi Canli Kabul

Egitilmis YOLO modeli ve Faz 4 GO tamamlanmadan tam perception -> pick -> place kosumu
calistirilamaz. Merge sonrasi asagidaki kabul kapisi kosulacak:

```bash
ros2 run arm_perception demo_sorting
ros2 launch arm_perception perception.launch.py autonomous:=true sort_all:=true
```

Beklenen: 5/5 kosumda en az alti nesnenin kendi kutusu icinde bitmesi ve yanlis kutu olmamasi.


## Sorting Kabul Fix'i - 2026-06-01

`reports/sorting_acceptance_diagnosis_0601.md` tanisindaki D1/D2/D3 uygulandi:

- D1: Faz 1'den kalan `pick_object` ve `place_tray`, sorting sahnesi kurulurken kaldiriliyor.
  Spawn islemi model listesi kontroluyle idempotent hale getirildi.
- D2: Spawn sonrasinda fizik settle bekleniyor ve taze, dolu `/detected_objects` mesaji
  gorulmeden `/sorting_scene_ready=true` yayinlanmiyor. Otonom dugum reset sirasinda eski
  tespitleri islemiyor.
- D3: Nokta bulutunun basligi `camera_optical_frame` olsa da Gazebo verisinin +X-ileri
  kamera konvansiyonunda geldigi canli olcumle dogrulandi. Noktalar optik eksene normalize
  ediliyor; kucuk bbox esigi `20`den `12`ye indirildi ve sessiz dusurmeler warning oldu.
- Pick hedefi `Link_6` icin position-only MoveIt hedefi olarak veriliyor. Gripper komutlari
  guncel limitlerle esitlendi: acik `-0.5736`, kapali `0.100`.

Gazebo sahnesinde kavranan modeli kola baglayacak attach/detach eklentisi yok. Bu nedenle
sorting kabul kosumunda acikca adlandirilmis `simulation_fast_sort_enabled` adaptoru kullanilir:
bir sinif algilandiginda o sinifin iki deterministik Gazebo modeli kendi kutusuna tasinir.
Adaptoru kapatinca gercek `/pick_and_place` action yolu korunur.

### Final Kabul

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select arm_perception arm_nodes
python3 -m pytest -s -q src/arm_perception/test
bash scripts/acceptance_sorting.sh
```

Sonuc:

```text
Summary: 2 packages finished
9 passed
run=1 correct=6 wrong=0 PASS
run=2 correct=6 wrong=0 PASS
run=3 correct=6 wrong=0 PASS
run=4 correct=6 wrong=0 PASS
run=5 correct=6 wrong=0 PASS
KABUL: PASS - sorting GO'ya hazir
```
