# Gerçek Donanıma Geçiş Planı + BOM — 6DOF Kol

> Amaç: simülasyondaki "beyni" (ROS2 + MoveIt2 + pick&place + perception/sorting)
> fiziksel 3D-baskı 6DOF kolda çalıştırmak. Strateji: ros2_control soyutlaması
> sayesinde **beyin değişmez**, yalnız en alttaki `hardware_interface` katmanı +
> URDF + kalibrasyon + perception extrinsics eklenir.

## Mimari (neyin değişip neyin sabit kaldığı)

```
Beyin (MoveIt + pick_place_node + perception/sorting)   ← SABİT (yeniden kullanılır)
   ↓ joint trajectory (donanımdan bağımsız)
ros2_control controllers (joint_trajectory_controller)  ← SABİT (config)
   ↓
hardware_interface (gerçek motor sürücüsü)              ← YENİ (tek donanıma özel parça)
   ↓
geri beslemeli akıllı servolar
```

---

## Faz planı

| Faz | İş | Çıktı | Süre (kaba) |
|----|----|-------|------|
| **0. Ölçüm + URDF** | Kolu ölç (link uzunlukları, eklem eksenleri, limitler); STL'lerden mesh. Gerçek URDF üret. | Gerçeği yansıtan `arm_description` URDF | 2–4 gün |
| **1. Motor + güç + bus** | Akıllı servoları tak, bus sürücü + ayrı güç. Her servo dönüyor + pozisyon okuyor mu doğrula. | Servolar ROS'tan komut alıyor + `joint_states` veriyor | 2–3 gün |
| **2. hardware_interface + kalibrasyon** | Hazır sürücüyü (Dynamixel/Feetech) ros2_control'e bağla. Her eklem: sıfır, yön, `rad↔tick` ölçek. FK gerçekle örtüşüyor mu (bilinen poza git, ölç). | Kalibre, doğrulanmış ros2_control | 3–5 gün |
| **3. MoveIt bring-up** | Gerçek URDF + gerçek controller'larla MoveIt. RViz'den jog, planla→çalıştır. Determinizm (OMPL preload) aynen geçerli. | Gerçek kolda planlı hareket | 2–3 gün |
| **4. Kamera + perception** | Kamera montajı + **kamera→taban extrinsic kalibrasyonu**. YOLO'yu gerçek ışık/nesnelerde ince ayar. | Gerçek algı → taban-çerçevesinde nesne pozu | 4–7 gün |
| **5. Pick&place + sorting** | Tam döngüyü çalıştır; hız/tolerans ayarı; erişim/eve-dönüş (Çözüm B) gerçeğe uygula. | Gerçek sınıf-bazlı sıralama | 3–5 gün |
| **6. Güvenlik + dayanıklılık** | Eklem/akım limitleri, **acil durdurma (e-stop)**, güç kesme rölesi, kablo düzeni. | Güvenli, tekrarlanabilir sistem | 2–4 gün |

**Toplam:** ~3–5 hafta (tek kişi, parçalar elde varsayımıyla). Beyin yeniden
yazılmıyor; iş **bring-up + kalibrasyon + ayar**.

---

## BOM (Bill of Materials)

### Seçenek A — Maliyet-etkin (ÖNERİLEN): Feetech akıllı bus servo
> LeRobot/SO-ARM ekosistemiyle aynı sınıf; geri beslemeli, tek seri bus, ucuz.

| # | Parça | Adet | ~Birim | ~Toplam | Not |
|---|-------|------|--------|---------|-----|
| 1 | Feetech **STS3215** akıllı servo (12V, ~30 kg·cm, pozisyon geri besleme) | 7 | $14 | $98 | 6 eklem + 1 gripper. Omuz/dirsek en zorlanan. |
| 2 | Bus servo sürücü (Waveshare Bus Servo Adapter / Feetech URT-1) | 1 | $10 | $10 | USB→seri bus; tüm servolar zincir. |
| 3 | Güç kaynağı 12V / 10A (servolar için, mantıktan AYRI) | 1 | $22 | $22 | Brown-out'u önler. |
| 4 | Raspberry Pi 5 (8GB) + soğutucu + SD + PSU | 1 | $110 | $110 | ROS2+MoveIt taşır. (Geliştirmede laptop yeterli.) |
| 5 | Derinlik kamera Intel RealSense **D435i** | 1 | $320 | $320 | Sim'deki RGB-D ile birebir. *Ucuz alternatif: RGB USB cam ~$30 (derinliksiz).* |
| 6 | E-stop buton + güç kesme rölesi | 1 | $15 | $15 | Güvenlik şartı. |
| 7 | Kablo/konnektör/DC jack/buck çevirici | — | $25 | $25 | Logic 5V için buck. |
| 8 | Mekanik (rulman, M2/M3 vida, kamera montajı) | — | $20 | $20 | Servolar 3D parçalara oturur. |
| | **TOPLAM (RealSense ile)** | | | **~$620** | RGB cam'le ~$330 |

### Seçenek B — Premium/sağlam: Dynamixel (resmi ROS2 desteği)
| # | Parça | Adet | ~Birim | ~Toplam | Not |
|---|-------|------|--------|---------|-----|
| 1 | Dynamixel **XL430-W250-T** (taban/omuz/dirsek — güçlü) | 3 | $50 | $150 | Yer çekimi torku yüksek eklemler. |
| 2 | Dynamixel **XL330-M288-T** (bilek ×3 + gripper) | 4 | $25 | $100 | Hafif distal eklemler. |
| 3 | **U2D2** + güç dağıtım hub | 1 | $60 | $60 | USB→Dynamixel bus. |
| 4 | Güç 12V / 5A | 1 | $20 | $20 | |
| 5–8 | Pi 5 + kamera + e-stop + kablo (A ile aynı) | — | — | ~$490 | |
| | **TOPLAM** | | | **~$820** | En sağlam, en pahalı. |

> Fiyatlar 2026 için **kaba** tahmindir; bölgeye/tedarikçiye göre değişir.

### EKSİKSİZ Premium BOM (Dynamixel — tam liste)
> Motor seçimi **tek voltaj (12V) uniform XL430** üzerine kuruldu: XL330 (5V) ile
> XL430 (12V) karışımı İKİ ayrı güç rayı gerektirir (data bus ortak ama güç değil),
> bu yüzden temiz/sağlam premium için hepsi XL430. (Daha ucuz karışık varyant en altta.)

| Grup | Parça | Adet | ~Birim $ | ~Toplam $ |
|---|---|---|---|---|
| **Eyleyici** | Dynamixel **XL430-W250-T** (6 eklem + gripper, uniform 12V) | 7 | 50 | 350 |
| | **U2D2** (USB↔Dynamixel TTL) | 1 | 50 | 50 |
| | U2D2 Power Hub kartı | 1 | 20 | 20 |
| | Dynamixel robot kablo seti (muhtelif boy) | 1 | 12 | 12 |
| **Güç+güvenlik** | Güç kaynağı **12V / 7A** (Dynamixel rayı) | 1 | 25 | 25 |
| | Buck 12V→5V (Pi/mantık yedek) | 1 | 6 | 6 |
| | E-stop buton | 1 | 8 | 8 |
| | Güç kesme rölesi/kontaktör (12V hattı) | 1 | 10 | 10 |
| | Sigorta + yuva (inline) | 1 | 5 | 5 |
| **Hesaplama+algı** | Raspberry Pi 5 (8GB) | 1 | 80 | 80 |
| | Aktif soğutucu | 1 | 7 | 7 |
| | microSD 64GB | 1 | 12 | 12 |
| | Pi güç adaptörü (27W USB-C) | 1 | 13 | 13 |
| | Intel RealSense **D435i** | 1 | 320 | 320 |
| | Güçlü USB hub | 1 | 15 | 15 |
| **Kablo+mekanik** | DC jack + güç dağıtım | — | — | 6 |
| | Lehim/makaron/kablo sarf | — | — | 10 |
| | Rulman seti | — | — | 12 |
| | M2/M3 vida-somun seti | — | — | 10 |
| | Kamera montaj donanımı | — | — | 6 |
| | Kablo düzeni (kılıf/klips) | — | — | 5 |
| | Filament (Dynamixel braket yeniden baskı, PLA/PETG) | 1 | 20 | 20 |
| | **TOPLAM (parça)** | | | **≈ $1.002** |
| | **+ Türkiye kargo/gümrük (~%30)** | | | **≈ $1.300** |

> **Karışık ucuz varyant:** XL430-W250-T ×3 (taban/omuz/dirsek) + XL330-M288-T ×4
> (bilek+gripper) → motorlar $150+$100=$250 (XL430×7'ye göre −$100). AMA XL330 5V,
> XL430 12V → ek 12V→5V güçlü regülatör (~$10) + iki rayın ayrı kablolanması gerekir.
> Toplam ≈ $912 (parça). Temizlik/sağlamlık için uniform XL430 önerilir.
>
> Not: omuz (joint_2) en yüksek yer çekimi torkunu taşır; kol ağırsa XL430 (1.5 N·m)
> yerine **XM430-W350** (~4.1 N·m, ~$70) yükseltmesi düşünülebilir (+$20/adet fark).

---

## Yazılım tarafı — ne yazılır / ne hazır
- **Hazır kullan:** `dynamixel_hardware` (Seçenek B) veya Feetech için topluluk
  ros2_control sürücüsü (Seçenek A) → `hardware_interface`'i sıfırdan yazmaya genelde
  gerek yok, config + kalibrasyon yeter.
- **Yeni/uyarlama:** gerçek `arm_description` URDF (Faz 0), ros2_controllers eşleme,
  kamera extrinsic, YOLO ince ayar.
- **At:** `sim_grasp_node` (sim'e özel tutuş hilesi) — gerçek gripper fiziksel tutar.
- **Aynen geçerli:** MoveIt planlama, OMPL determinizm (preload), pick_place state
  machine, sorting mantığı, Çözüm B (erişim/eve-dönüş).

## Kritik riskler / dikkat
- **Güç:** servoları asla mantık 5V'undan besleme → ayrı kaynak + ortak GND.
- **URDF doğruluğu:** yanlış ölçü → kol yanlış yere gider (en sık hata).
- **Açık vs kapalı döngü:** "doğru motorlar" = geri beslemeli (yukarıdaki ikisi de).
  Düz hobi PWM servo (geri beslemesiz) seçilirse beyin körlemesine çalışır.
- **Güvenlik:** gerçek kol insana/eşyaya çarpar → e-stop + yazılım limitleri şart.
