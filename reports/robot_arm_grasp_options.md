# Robot Arm Kavrama & Gripper Modeli — Analiz ve Seçenekler

Tarih: 2026-07-05
Hazırlanma nedeni: kullanıcı talebi — "önce analiz et"

## 0. Tetikleyen iki gözlem (kullanıcı, 2026-07-05, canlı GUI)

1. **"Gerçek bir kavrama olmadan kırmızı nesne direkt kutuya yerleştirildi."**
2. **"`buyuk_servo` (gripper motoru) parmakları istediğim gibi açıp kapatmıyor."**

Bu iki sorun **aynı köke** çıkıyor: Robot Arm gripper'ı sim'de gerçek bir sıkıştırma
(pinch) yapamıyor, bu yüzden yerleştirme fizik yerine ışınlamayla (`set_pose`)
taklit ediliyor.

## 1. Mevcut durum — gripper modeli

- `link_6.stl` (~173 KB) = **tüm hareketli gripper takımı (iki parmak + servo
  boynuzu) TEK rijit mesh**.
- `joint_6` = bu tek gövdeyi döndüren **tek `continuous` eklem**
  (`robot_arm_body.xacro`, `robot_arm_joint` makrosu; tüm eklemler continuous, limit
  yok). Eksen `0 0 -1`.
- MoveIt SRDF: `gripper` grubu yalnız `joint_6`; `open`=+0.3 rad, `closed`=−0.5 rad
  (`robot_arm.srdf`).
- Controller: `robot_arm_gripper_controller` (JTC) tek eklem `joint_6`, position arayüzü.
- Kalibrasyon: `servo_calibration.yaml` joint_6 = gripper servosu, `min_rad=-1.57`
  (yorum: "aralık gripper aç/kapa stroke'una göre daralacak" — henüz daralmadı).

### Sonuç
- "Aç/kapa" fiilen **tüm elin joint_6 etrafında dönmesi**. İki parmak birbirine
  göre HİÇ hareket etmiyor → nesneyi sıkıştıramaz.
- `continuous` tip yanlış: gripper servosunun sınırlı stroke'u var; model sonsuz
  dönebilir, açık/kapalı fiziksel limiti yok.
- Bu yapıda Gazebo fiziğiyle nesne tutmak imkânsız (temas eden hareketli parmak
  çifti yok) → bu yüzden `autonomous_pick_node` `simulation_post_place_sync` ile
  nesneyi action başarısında kutuya `set_pose`'luyor (ışınlama).

## 2. Mevcut yerleştirme (teleport-sync) — ne kanıtlıyor, ne kanıtlamıyor

- ✅ Kanıtlıyor: perception (YOLO) → IK → MoveIt planlama → yaklaşma/place
  hareketleri → sınıf→kutu eşleme mantığı → sahne akışı. 3/3 koşu × 6/6, 0mm
  (ışınlama olduğu için 0mm).
- ❌ Kanıtlamıyor: gerçek fiziksel kavrama, parmak stroke'u, tutma kuvveti,
  nesnenin taşınırken düşüp düşmediği.
- Gerçek kolda ışınlama yok → M2/M4'te ilk kez gerçek kavrama denenecek; sim şu an
  bu riski azaltmıyor.

## 3. Eski kol DetachableJoint saga'sından dersler

(2026-06-13 düzeltme turu)

- gz-sim 8.11 `DetachableJoint` **sim başında otomatik attach** ediyordu → nesne
  spawn anında Link_6'ya ~0.6m offsetle rijit kaynaklanıp uçuyordu.
- `suppress_initial_attach` bu gz-sim sürümünde `.so`'da YOK.
- Çözüm zor oldu: explicit `<attach_topic>`/`<detach_topic>` + runtime joint kurma
  + kapanış kenarında attach / açılış kenarında detach.
- `sim_grasp_node` uzun süre "ready"den sonra tetiklenmedi (executor tight-loop,
  timer/joint_states ateşleme sorunları).
- Nihayetinde kullanıcı **pick/place'i fizikten decouple etme** kararı verdi
  (2026-06-24) — deterministik demo için.

Ders: **DetachableJoint tabanlı gerçek kavrama gz-sim 8'de kırılgan ve pahalı**;
ama teleport da gerçek grasp'ı hiç test etmiyor. Orta yol: parmakları doğru
modelleyip **sürtünme tabanlı** kavrama (fizik) veya kontrollü DetachableJoint.

## 4. Seçenekler

Her seçenek ÖNCE **iki-parmak modelleme düzeltmesini** gerektirir (Adım 0):

### Adım 0 (tüm gerçek-grasp yollarının ön koşulu): parmakları modelle
- `link_6.stl`'i CAD'de böl: **avuç (palm, rijit, link_5/6'ya sabit)** + **2
  parmak link'i** (finger_left, finger_right).
- `joint_6` → sol parmak sürücü eklemi (**revolute + limit**, continuous DEĞİL);
  sağ parmak `mimic` (joint_6, multiplier=-1) veya ikinci sürücü.
- Parmak uçlarına düzgün **collision geometrisi** (kutu/silindir primitifleri;
  STL collision pahalı ve delikli olabilir).
- SRDF open/closed değerlerini gerçek stroke'a göre yeniden ayarla.
- Efor: **orta** (CAD bölme + STL yeniden export + xacro/SRDF/controller güncelleme
  + gain ayarı). Kullanıcı CAD erişimi gerekir.

### Seçenek A — Sürtünme tabanlı fizik kavrama (önerilen gerçek-grasp yolu)
- Adım 0 sonrası: iki parmak nesneyi fiziksel olarak sıkar; gz-sim sürtünmesi
  nesneyi tutar. DetachableJoint YOK.
- Artı: en gerçekçi; gerçek kola en yakın; tutma kuvveti/stroke doğrulanır.
- Eksi: gz-sim temas/sürtünme ayarı hassas (kayma, titreme); parmak collision +
  nesne sürtünme katsayıları + gripper gain deneysel ayar ister. Küçük nesnede
  (~2cm) zor.
- Efor: **yüksek**.

### Seçenek B — Kontrollü DetachableJoint (Adım 0 + attach/detach)
- Adım 0 sonrası: kapanış kenarında parmak↔nesne fixed joint (attach), açılışta
  detach. Eski koldaki explicit topic yaklaşımı.
- Artı: deterministik tutma; sürtünme cehennemi yok; taşırken düşmez.
- Eksi: eski saga'nın kırılganlığı; hâlâ "gerçek kuvvet" değil (rijit kaynak);
  başlangıç auto-attach tuzağı.
- Efor: **orta-yüksek**.

### Seçenek C — Mevcut teleport-sync'i koru (grasp'ı yalnız donanımda doğrula)
- Sim = pipeline/planlama doğrulaması; grasp fiziği M2/M4'te gerçek serviyle.
- Artı: sıfır ek iş; demo deterministik.
- Eksi: sim en kritik donanım riskini (kavrama) test etmiyor; kullanıcının
  "eksiksiz çalışsın" hedefini karşılamıyor; parmak modeli hâlâ bozuk (görsel
  olarak yanlış — el rijit dönüyor).

### Not: Parmak görselini düzeltmek grasp'tan bağımsız da değerli
Seçenek C'yi seçse bile **Adım 0'ın görsel kısmı** (iki parmağın gerçekten
açılıp kapanması) yapılmalı — aksi halde canlı GUI'de el rijit blok olarak döner,
kullanıcının bildirdiği sorun sürer.

## 4b. UYGULANDI — Adım 0 (Yol 2 idealize), 2026-07-05

Kullanıcı "Yol 2 — idealize primitif, hemen başla" + "parmaklar pivot (revolute)"
dedi. YAPILAN:
- `link_6` → **rijit palm** (link_5'e sabit, eski joint_6 origin'i). Görsel STL korundu.
- `joint_6` → **revolute** sürücü sol parmak (finger_left, primitif kutu, limit
  [-0.6,0.4], axis -Z). İşaret uzlaşımı korundu: **+0.3 açık / -0.5 kapalı**.
- `finger_right` → simetrik primitif kutu, `joint_6_mirror`.
- Geometri palm frame'inde tool0 grasp merkezine (-0.082,0,0) nişanlı.

### KRİTİK BULGU — gz-sim mimic desteklemiyor
`<mimic>` ile finger_right sadece **RSP/TF** seviyesinde takip etti; **Gazebo
fiziğinde kıpırdamadı** (`[Err] Physics.cc: chosen physics engine does not support
mimic constraints`). Yani RViz doğru ama **kullanıcının izlediği Gazebo GUI'de tek
parmak** hareket ediyordu. ÇÖZÜM (varyant-koşullu):
- **SİM:** `gripper_sim=true` → joint_6_mirror mimic DEĞİL, **ikinci sürücü eklem**
  (sim ros2_control + `robot_arm_gripper_controller` joints=[joint_6, joint_6_mirror];
  `robot_arm_pick_place_node` yeni `gripper_mirror_joint` param'ıyla q ve -q gönderir).
- **GERÇEK:** `gripper_sim=false` → joint_6_mirror **URDF mimic(-1)** (tek servo iki
  parmağı sürer; RSP TF için; STM32'ye gitmez).

### Doğrulama (2026-07-05)
- gz-tarafı poz: OPEN→CLOSED'da HER İKİ parmak oryantasyonu zıt yönde değişti
  (finger_left z −0.169→+0.200; finger_right z +0.109→−0.260) — simetrik fiziksel
  hareket ✓. Mimic hatası kayboldu.
- Stroke ölçümü (TF, tip): açık gap **62.6mm**, kapalı **7.1mm**.
- **Regresyon:** yeni gripper + 2-eklem komut yoluyla tam Robot Arm sorting **6/6, 0mm,
  0 gripper reddi, 0 fonksiyonel hata** — pipeline bozulmadı.
- Sim ve real xacro ayrı ayrı parse PASS (sim: mimic yok/aktif; real: mimic var).

Kalan: gerçek servo stroke açıları (aç/kapa) → SRDF open/closed + kalibrasyon
`min/max_rad` daraltılacak (kullanıcı ölçümü). Fizik kavrama (Seçenek A) hâlâ açık
— artık iki parmak fiziksel hareket ettiği için uygulanabilir.

## 4c. GERÇEK PARMAK MESH'İ ENTEGRE (2026-07-06)

Kullanıcı gerçek gripper'ı verdi: `robot_arm_gripper/.../Gripper 1.STL` (tek
parmak; iki parmak AYNI parça) + önlü/arkalı render. Idealize kutular kaldırıldı.
- **Ölçü (STL parse):** 65.3 × 20.4 × 8.5 mm, kavisli. İki delik: üst (pivot)
  STL(4.75,62.1)mm, alt (37.9)mm; bore = X ekseni.
- **Montaj:** üst delik = pivot. STL→link eksen dönüşümü (bore→palm Z, uzunluk→−X
  grasp'a doğru); mesh scale mm→m. `gripper_finger.stl` meshes/'e kopyalandı,
  visual+box collision. finger_right = AYNI parça Rx(π) döndürülmüş (identical part).
- **Geometri (previz + URDF render doğrulandı):** iki kavisli parmak birbirine bakar,
  uçlar tool0 grasp merkezine (−0.082) nişanlı; pivot (px=−0.030, py=±0.016).
- **Stroke (küçüldü):** joint_6 limit [−0.30,0.20]; **open=+0.12 / closed=−0.08**
  (2cm nesneyi makaslamadan kavrar; SRDF + robot_arm launch param). Kesin servo açıları
  kullanıcı ölçümüyle daralacak.
- **Doğrulama:** URDF-render previz'le birebir; tam sorting regresyon **6/6, 0 gripper
  reddi, 0 mesh hatası, 0 fonksiyonel hata**.
- **UYARI (dürüstlük):** palm/servo montaj transformu (pivotun wrist üzerindeki tam
  yeri) kullanıcı yalnız PARMAK STL'i verdiği için TAHMİN — parmak şekli/ölçüsü/
  kinematiği gerçek, ama gripper'ın link_5 üzerindeki tam konumu göz kararı. Kullanıcı
  GUI'de bakıp nudge edebilir (palm_joint origin veya pivot px/py).

## 5. Teknik öneri

1. **Adım 0'ı her hâlükârda yap** — parmak modelleme boşluğu hem görsel hatanın
   hem grasp imkânsızlığının köküdür. Kullanıcının CAD'de `link_6`'yı palm + 2
   finger'a bölmesi (veya mevcut STL'i parçalama) gerekir.
2. Sonra **Seçenek A (sürtünme fizik)** hedefle; çok kırılgan çıkarsa **B**'ye düş.
3. Teleport-sync'i A/B çalışana kadar fallback olarak koru (regresyon güvencesi).

## 6. Kullanıcı girdileri + Adım 0 planı (güncel 2026-07-05)

### Kesinleşen
- **Parmak kinematiği = PIVOT (revolute)** (kullanıcı, 2026-07-05). Makas/pens tipi:
  servo horn bir parmağı çevirir, diğeri simetrik. → `joint_6` sürücü parmak
  revolute+limit, ikinci parmak `mimic(joint_6, multiplier=-1)`.

### İki geometri yolu (Adım 0)
- **Yol 1 — CAD split (yüksek görsel doğruluk):** kullanıcı `link_6`'yı palm +
  finger_L + finger_R olarak ayrı STL export eder + parmak pivot koordinatları.
  Gerçek mesh görünümü korunur.
- **Yol 2 — İdealize primitif (CAD beklemeden, ÖNERİLEN başlangıç):** proje sahibi
  link_6 STL zarfına oturan **primitif kutu parmaklar** (palm rijit + 2 revolute
  parmak) üretir. Görsel şık değil ama: (a) parmaklar GERÇEKTEN açılıp kapanır
  (kullanıcının bildirdiği görsel hata çözülür), (b) collision primitifleriyle
  fizik kavrama (Seçenek A/B) hemen denenebilir. Gerçek kolda kavramayı belirleyen
  şey mesh estetiği değil parmak DOF'u + stroke; primitif bunu tam test eder.
  CAD sonra Yol 1 ile değiştirilebilir.

### Hâlâ gereken girdi
- Gerçek stroke: tam açık ve tam kapalı servo açıları (kalibrasyon `min/max_rad`).
  Bilinmiyorsa ilk doğrulama makul bir aralıkla (örn. açık +0.35 / kapalı 0.0 rad,
  parmak ucu ~2cm nesneyi saracak şekilde) başlar, kullanıcı ölçünce daraltır.
- (Yol 1 seçilirse) palm/finger STL'leri + pivot xyz.
