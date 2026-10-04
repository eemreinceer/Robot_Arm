# Sorting Kabul Testi — Tam Teşhis Raporu (2026-06-01)

Sabah dinlenmiş sistemde (load ~1, eğitim bitmiş) 5 sonda koşuldu.
Kabul kapısı HÂLÂ GEÇMEDİ; ama kök neden artık **kesin**.

## TL;DR
Sorting demosu **Faz 1'in `pick_and_place.world` dosyasını** kullanıyor. O world'de
zaten gömülü iki sabit obje var: yeşil `place_tray` (0.1/0.6/0.1) ve kırmızı
`pick_object`. `demo_sorting` 6 sorting nesnesini bunların ÜSTÜNE spawn ediyor →
sahne karışıyor, perception eğitilmediği yeşil tepsiyi görüyor, `/detected_objects`
pratikte boş kalıyor, kol hiç pick yapmıyor. Dünkü "perception boş" + "pick yok" +
"Gazebo çöktü" semptomlarının ortak kök nedeni budur (Gazebo çökmesi ayrıca gece
kaynak darlığıydı — bugün çökmedi).

## Çözülmüş (launch/bringup alanı)
1. `/ik_solve` yoktu → arm_bringup/perception.launch.py'ye `kinematics_node` eklendi. ✅ IK PASS.
2. `sort_all` yutuluyordu → arm_bringup launch'ına argüman + node param wiring. ✅ mode=sorting.
3. arm_bringup/package.xml'e arm_kinematics + arm_perception exec_depend. ✅
   (Bu 3 fix working tree'de + install'e yansımış, henüz commit edilmedi.)

## Eğitim (sağlam)
YOLOv8n, 80 epoch, test mAP50=0.995 / mAP50-95=0.9855. Model
`src/arm_perception/models/yolo_arm.pt`. Bağımsız testte canlı kamera görüntüsünde
`red_box`'ı 0.94 conf ile buldu → model İYİ, sorun modelde değil.

## Sonda bulguları
- **Gazebo:** dinlenmiş sistemde 90s gözlemde çökmedi. Gece çökmesi = eğitim load'u (8+) altında kaynak darlığı, kod bug'ı değil.
- **Kamera:** /camera/image ~4-5 Hz akıyor; /camera/points organize (480×640) ve akıyor (perception'ın `latest_points` şartı sağlanıyor).
- **Model canlı:** kaydedilen frame'de (reports/sorting_cam_frame_diag.png) sadece 1 kırmızı kare + büyük yeşil blok + gri çubuk görünüyor. conf 0.01'e indirildi → yine tek `red_box:0.94`.
- **Projeksiyon (offline, fx=554.38):** spawn edilen 6 nesnenin HEPSİ teorik olarak frame İÇİNDE (u:259-407, v:212-300). Yani geometri/FOV doğru — nesneler görünmeli.
- **World analizi:** `src/arm_gazebo/worlds/pick_and_place.world` include ediyor:
  - `pick_object` @ (0.3,0,0.65) — kırmızı kutu (görüntüdeki kırmızı kare = bu, sorting nesnesi değil)
  - `place_tray` @ (0.45,0.25,0.61) — YEŞİL tepsi 0.2×0.3 (görüntüdeki büyük yeşil blok = bu)
- Sorting nesne SDF renkleri DOĞRU (red 0.8/0.1/0.1, yellow 0.8/0.8/0.1, blue 0.1/0.1/0.8) — yani gri/yeşil görünenler sorting nesneleri değil, world'ün gömülü objeleri.

## Kök neden
`demo_sorting` temiz bir sorting world'ü yerine Faz 1 world'ünü kullanıyor; gömülü
pick_object + place_tray sahneyi kirletiyor. Ayrıca spawn edilen nesnelerin
render/settle olması beklenmeden frame yakalanıyor olabilir (spawn z=0.62-0.625,
masa üstü z=0.6; düşme/sekme ihtimali). Net düzeltme alanı = perception/sorting
sahne = perception pipeline.

## Önerilen düzeltme
1. Sorting için gömülü pick_object/place_tray İÇERMEYEN temiz bir world (veya bu
   ikisini sahne kurarken kaldırmak). demo_sorting --world / launch world argümanı.
2. Spawn sonrası settle + render beklemesi; perception'ın ilk dolu /detected_objects
   mesajını üretmesini doğrulayan bir gate.
3. perception_node poz aşamasında nesne düşürme: küçük bbox → <minimum_cluster_points
   (20) → continue. Eşik/again kontrolü; gerçekten görülen nesnenin yayına girdiğini doğrula.

## Durum
Ek paralel çalışma başlatılmadı. Fix'ler commit
edilmedi. Bir sonraki adım kullanıcı kararına bırakıldı.
