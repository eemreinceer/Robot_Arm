# Sim Grasp (gerçek hareketli tutuş) — WIP / PARK NOTU

Tarih: 2026-06-02

## Amaç

Sorting kabul testinin "5/5" sonucu, FİZİKSEL tutuşu değil yalnız algı + sınıf→kutu
mantığını doğruluyor: varsayılan `simulation_fast_sort_enabled=True` ile nesneler
`gz set_pose` ile kutuya **ışınlanıyor**, kol hiç hareket etmiyor. Hedef: kolu gerçek
`/pick_and_place` hareketiyle koşturmak ve nesneyi sim'de gerçekten taşımak.

## Mimari karar (korunuyor)

Tutuş mantığı otonomi BEYNİNİN DIŞINDA tutulur. Beyin (autonomous_pick_node /
pick_place_node) sadece gripper'ı normal kontrolcüden kapatır/açar; sim'e özgü tutuş
ayrı bir yalnız-sim düğümle sağlanır: **`src/arm_gazebo/scripts/sim_grasp_node.py`**.
Bu düğüm gripper eklemini izler, kapanınca `grasp_link`'e en yakın `sorting_*` modelini
kinematik olarak kilitler (kola özgü değil; gerçek donanımda hiç başlatılmaz). Böylece
"her kola takılan beyin" prensibi korunur — bu, atılabilir bir sim katmanıdır.

Durum: düğüm yazıldı, `arm_gazebo` CMakeLists'e kuruldu, `arm_bringup/perception.launch.py`
içinde `autonomous` koşuluyla wire edildi (sim_grasp_node, period=24s). Build temiz,
İlgili 9 unit test geçiyor.

## Neden WIP (3 koşu teşhisi)

`simulation_fast_sort_enabled=false` + sim_grasp_node ile 3 kabul koşusu:
- Algı + sahne sağlıklı (6 nesne, detection gate PASS).
- **Çözüldü:** poz okuma `dynamic_pose/info` (yalnız hareketliler) → `pose/info` (tümü);
  grasp_link dünya pozu base-ofset varsayımı yerine doğrudan `world→grasp_link` TF'inden
  (z artık gerçekçi ~0.647).
- **Açık kalan blokerler (her biri ayrı ~6 dk döngü ister, transfer değeri YOK):**
  1. Gripper kapanma anında `grasp_link` nesnenin üstünde değil (`(0.659,0.192)` vs nesne
     `x~0.3-0.45`, en yakın 0.25 m) → kol amaçlanan pick pozuna tam varmıyor veya
     Link_6↔grasp_link offset / kapanma zamanlaması uyumsuz.
  2. Gripper pick'ler arası tam açılmıyor → eşik temiz geçilmiyor (koşuda ~4 pick'e karşı
     yalnız 2 "kapanma" kenarı).
  3. Beyin-dışı, ön-var sorun: MoveIt place/bin pozlarına planlamayı sık sık başarısız
     ("Planning failed", `approaching_place` abort) + pick başına ~1-3 dk (yavaş).

## Karar

Gerçek-hareketli sim grasp **kasıtlı olarak PARK edildi.** Gerekçe: bu sim artefaktının
gerçek donanıma transfer değeri yok (gerçek gripper fiziksel sürtünmeyle tutar; bu
düğüm hiç çalışmaz). Kabul testi varsayılan ışınlama yoluna geri alındı (5/5 PASS,
yeşil hali). sim_grasp_node + wiring ileride istenirse diye repoda WIP olarak duruyor.

Devam edilmek istenirse başlangıç noktası: yukarıdaki blok #1 (pick-poz hizası) —
`sim_grasp_node` grasp anında `grasp_link` vs gerçek pick_pose'u loglar.
