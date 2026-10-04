# Kol başındaki bir sonraki oturum — plan

**Yazıldı:** 2026-08-14, fiziksel erişimsiz geçen günün sonunda.
**Kime:** kolun başına geçen kişiye. Operatör başında, kesici elde.

Sıra **riski artan** şekildedir. Her adımın bir çıktısı var; **çıktı gelmeden
sonrakine geçilmez.** Pasif ölçümler, kolu hareket ettiren tek adımdan önce
gelir — bu kural bu depoda bir kez ihlal edildi ve düzeltildi (`7d23c09`).

---

## 0. Fiziksel durumu ÖLÇ, varsayma — 5 dk, HAREKET YOK

Kola dokunmadan önce dört şey ölçülür: ray gerçekten kesik mi, stack ayakta mı,
`armed` ne durumda, `/dev/ttyTHS1` sahipsiz mi.

```bash
docker ps
ros2 topic hz /joint_states
ros2 param get /robot_arm_hardware_safety armed
```

**Neden bu adım listede:** 2026-08-04'te tam tersi varsayılmıştı. Üç ayrı değerlendirme
boyunca "ray kesik, stack durmuş" yazıyordu; ölçülünce `/joint_states` **50 Hz**
akıyordu ve `armed=True`'ydu. Yazılı bir varsayım, ölçüm değildir.

**Çıktı:** başlangıç durumunun yazılı kaydı. Bu olmadan sonraki hiçbir ölçüm
"neye göre" sorusuna cevap veremez.

---

## 1. ESP32 flash + kalibrasyon kapısı — 20 dk, RAY KESİK

Ray kesikken yapılır; servo dönmez.

**Flash (PC'den, DevKit'in USB'si takılı):**

```bash
cd firmware/esp32_servo_ctrl && pio run -e esp32dev -t upload
```

Birden çok seri port varsa `--upload-port /dev/ttyUSB0` ekle. Build **zaten
korumalı**: `platformio.ini` her derlemeden önce `generate_robot_config.py
--check` koşuyor, yani üretilmiş header YAML'dan bayatsa derleme durur. Bayatsa
önce üreteci koştur, sonra flash'la.

**⚠ DOĞRULAMA USB'DEN YAPILAMAZ.** ESP32'nin **UART0'ı boot konsoludur** ve her
reset'te oraya banner basar; protokol bilerek **UART2**'ye alınmıştır
(`platformio.ini` başındaki not). USB üzerinden `V?` sorarsan boot log'una
konuşmuş olursun. Doğrulama, ROS hattının bağlı olduğu yerden — **Nano'nun
UART'ından** — yapılır:

```bash
# Nano'da (ray KESİK):
python3 scripts/verify_calibration_gate.py --device /dev/ttyTHS1
```

Bu araç `V?` el sıkışması → `K?` → parmak izi karşılaştırması yapar ve tek satır
PASS/FAIL basar. Çıkış kodları: 0 PASS, 1 uyuşmazlık, 2 port açılamadı,
3 doğrulanamadı.

**Beklenen:** `PASS`. Host ve firmware aynı YAML'dan türüyor, beklenen parmak
izi `21aecfe95643057d`.

**Beklenmiyorsa:**
- *mismatch* → firmware başka bir kalibrasyondan yakılmış; üreteci koştur,
  reflash et. Aracın var olma sebebi tam olarak budur.
- *unsupported* (`E1`) → flash tutmamış; eski firmware koşuyor.

**Çıktı:** kapının ilk canlı kanıtı. Sonra `require_calibration_match:=true`
yapılır ve bir daha yanlış eşleşmeyle kol sürülemez.

---

## 2. Watchdog kararı — 30 dk, RAY KESİK, TEK SERVO

D2 önerisi (aşağıdaki belge) okunur, karar verilir, sonra tezgahta doğrulanır: komut
akışını kes, PWM'e ne olduğuna **bak** (log'a değil, sinyale).

**Bugün ölçülen durum:** `P` kanalının watchdog'u (1000 ms) yalnız bir kez `E3`
basıyor, **PWM latch'li kalıyor**; smooth yolun timeout'u da `PWM=HOLD`. Yani
**Jetson ölürse tork süresiz kalır** ve `E3`'ü ROS tarafında okuyan yok.

**Öneri hazır:** `docs/watchdog_policy_proposal.md` — üç seçenek, gerekçeleri ve
tezgah doğrulama planı. Kısaca: önerilen **C (tut + duyur)**; fiziksel davranış
aynı kalır (frensiz kolda PWM kesmek kolu düşürür) ama sessizlik kalkar.
**B (nötre al) encoder gelmeden savunulamaz** — "güvenli duruş" tanımlı değil.

Frensiz bir kolda "tut" savunulabilir — kesmek kolu düşürür. Ama **seçilmiş**
olmalı, miras kalmış değil.

**Çıktı:** karar + kanıt. Kolu ilk kez uzun süre enerjili bırakacağın günün ön
koşulu budur.

---

## 3. Taban ölçümü — 2–3 saat, HAREKET VAR

Buradan sonrası kol hareket ediyor: küçük adım, hız tavanı, kesintide q=0,
operatör kesici elde. D3 araçlarıyla üç ölçüm:

> ⚠ **Bu adım 2026-08-14 gecesi yeniden yazıldı.** Önceki sürüm "taban poz
> çeşitliliğiyle büyüyor, sebebi FK/servo hatası" varsayımına dayanıyordu; o
> açıklama çürüdü (bkz. `docs/hand_eye_pose_set_requirement.md` son bölüm).
> Ölçüldü: baskın kanal tahtanın **oryantasyon** hatası, öteleme değil.
> 0.41 mm öteleme tek başına 0.45 mm X hatası veriyor; yalnız 0.5° dönme
> 2.94 mm veriyor (gerçekte ölçülen 2.81 mm).

**3a. Açısal tabanın gerçek koldaki değeri.** Sim'de tahtanın oryantasyon
tutarsızlığı p90 0.46°. Gerçek kolda bu sayı ne? Aynı pozdan tekrar tekrar kare
al (kol sabit), tahtanın pozunu kaydet. Kol hareket etmediği için bu **saf algı
gürültüsüdür** — servo/FK karışmaz.

**3b. Mekanik payı.** Aynı pozu farklı yaklaşma yönlerinden N kez ziyaret et.
3a'nın üstüne çıkan fark **mekaniktir** (servo tekrarlanabilirliği + FK).
İkisinin farkı, "hedefe algı tarafından mı mekanik taraftan mı yaklaşılmalı"
sorusunu ayırır.

**3c. Flip taraması.** Havuzun ~%14'ü düzlemsel PnP çift-çözüm belirsizliğine
düşüyor (100–215 mm, reprojeksiyon düşük). Gerçek karelerde bu oran ne?
`scripts/hand_eye_floor_analysis.py` aynı analizi gerçek yakalamaya uygular.

> **Çevrimdışı prova (2026-08-15).** `repeatability_plan.py` (3b) ve
> `flip_sweep.py` (3c) de sentetik kamerayla koşuldu.
>
> **3b sağlam.** `plan` dört yaklaşma yönü üretiyor; `analyze` pozitif ve negatif
> kontrolde de doğru hüküm veriyor — mekanik pay enjekte edilmediğinde
> "ölçülebilir değil, iş algı tarafında", 1.5 mm / 0.35 derece enjekte
> edildiğinde "fark mekaniktir". Not: hüküm yalnız AÇISAL fazlalığa bakıyor;
> öteleme fazlalığı yazdırılıp değerlendirilmiyor. Açısal kanalın baskın olduğu
> bulgusuyla tutarlı, ama bilinsin.
>
> 🔴 **3c'de yapısal bir bulgu: flip marjı simetriden dolayı SIFIR.** 6x9 iç köşe
> ızgarası 180 derece dönme altında kendine eşleniyor, yani ters sıralamanın
> reprojeksiyonu native ile birebir aynı çıkıyor — gürültü bu berabereliği
> bozmuyor (sigma=4 gri seviyede bile margin 0.0000, kareler %100 ambiguous).
> Dolayısıyla `board_pnp`'nin "ters sıralamayı ancak gerçek bir marjla daha
> iyiyse seç" kuralı hiç tetiklenemez (120/120 native seçildi). Tahta kendi
> düzleminde dönerken native sıralamanın kendisi 120 karede 21 kez döndü ve bu,
> marj kuralıyla tespit edilemez.
>
> Pratik sonuç: **pozlar arasında düzlem-içi dönme farkını küçük tut.** Kareler
> arası büyük in-plane dönme, sessizce yanlış eşleşmiş köşe kümesi üretebilir.
> Kalıcı çözüm ChArUco'dur (her köşenin kimliği var, simetri kırılır) — bu,
> `hand_eye_pose_set_requirement.md`'nin zaten önerdiği yol ve bu ölçüm ona
> yapısal bir gerekçe ekliyor.

**Çıktı:** açısal tabanın gerçek değeri ve algı/mekanik ayrımı. Sıradaki fazın
yönünü belirleyecek ölçüm budur.

### ✅ Araçlar çevrimdışı prova edildi (2026-08-15) — kolun başında ilk kez koşmayacak

`measure_perception_floor.py` bu tarihe kadar yalnız sözdizimi ve hata yolu
bakımından kontrol edilmişti; ilk gerçek koşusu kolun başında olacaktı.
`scripts/fake_board_camera.py` eklendi: bilinen pozdaki board'u `/camera/image_raw`
+ `/camera/camera_info` üzerinden yayınlar, yani zincir **yer gerçeğine karşı**
sınanabilir — gerçek kamerada olmayan bir imkân.

Sonuç: 20/20 kare kabul, reprojeksiyon 0.073 px, ve çözülen poz yer gerçeğine
**0.033 mm / 0.048 derece** oturuyor. Saçılım hesabı gürültüye tepki veriyor
(gürültüsüzde 0.000, sigma=6 gri seviyede 0.006 mm / 0.006 derece), yani
takılıp sıfır basmıyor.

Prova üç kusur buldu; üçü de kolun başında çıkacaktı:
- Artifact board'un **pozunu kaydetmiyordu**. 3b iki ziyareti karşılaştırmak
  zorunda ve saçılım tek başına onları ilişkilendirmiyor; ziyaretler tekrar
  edilmek zorunda kalırdı. Eklendi.
- Sentetik render'da köşeler sistematik olarak (-0.5, -0.5) px kayıyordu
  (tam sayıya yuvarlanmış doldurma) ve reprojeksiyon 1.20 px'e çıkıyordu, yani
  1.0 px kapısı HER kareyi reddediyordu. Alt-piksel doldurmayla düzeldi.
- `fake_board_camera` Ctrl-C'de çift `rclpy.shutdown()` ile hata basıyordu.

> 🔴 **TAHTAYA TAM KARŞIDAN BAKMA.** Ölçüldü 2026-08-15: düzlemsel PnP
> fronto-paralel görüşte dejenere. Eğim taraması (gürültüsüz, sentetik):
>
> | Eğim | 0 | 1 | 2 | 5 | 12 | 30 derece |
> | --- | ---: | ---: | ---: | ---: | ---: | ---: |
> | Rot hata (derece) | **1.77** | 0.30 | 0.07 | 0.04 | 0.006 | 0.009 |
>
> Ve 0 derecede **reprojeksiyon 0.671 px** — aracın 1.0 px kapısının ALTINDA.
> Yani kapı, 1.77 derecelik hata taşıyan kareyi sessizce kabul eder. 0.5 derece
> yaklaşık 2.9 mm X hatası olduğuna göre bu tek başına hedefi ıskalatır.
>
> Bu, issue #8'deki "tahta saçılımı tek başına kabul kapısı olamaz" bulgusunun
> kardeşidir: **reprojeksiyon bu hatayı göstermiyor.** Poz kümesi tahtaya dik
> bakan görüşlerden kaçınmalı; birkaç derece yetiyor, ama pay bırakmak için
> 10 derece ve üstü makul.

---

## 4. KARAR NOKTASI — buraya gelmeden hand-eye koşusu YAPMA

3'ün sonucuna göre iki yol var ve **seçim kullanıcınındır**:

- **Taban küçülebiliyorsa** (ör. hata baskın olarak `zero_offset`'ten geliyorsa
  ve düzeltilebiliyorsa): tabanı düşür, sonra 20 pozluk kümeyle gerçek hand-eye
  koşusu yap. Küme hazır: `data/hand_eye/gated_pose_set.json`.
- **Taban donanımın tavanıysa:** 1.0 mm hedefi bu donanımla ulaşılamaz demektir.
  (Bugünkü kayıt: `data/hand_eye/gated_pose_set.json` düzeltilmiş modelle
  beklenen hatayı **ort 2.69 / p90 4.43 mm** diye yazıyor, verdict KALDI;
  gerçek veride ölçülen 2.81 mm. Eski öteleme-tabanlı sayı 0.85 mm idi ve
  yanıltıcıydı.)
  O zaman hedef **gerekçesiyle yeniden ilan edilir** (ör. 3 mm) — **koşudan
  önce.** `docs/hand_eye_pose_set_requirement.md` bunu açıkça yazıyor: koşu
  sonrası hedefin gevşetilmesi yasaktır.

### Neden hand-eye koşusu 3'ten önce yapılmamalı

Beklenen hata ~2.8 mm, ilan edilen hedef 1.0 mm. Şimdi koşarsan elinde hedefi
tutmayan bir sonuç olur ve onu "kabul mü etsem" diye tartışırsın — yani kapıyı
koşudan sonra gevşetme baskısı doğar. Önce tabanı ölç, sonra ya düzelt ya da
hedefi dürüstçe yeniden ilan et.

---

## Süre

0–2 arası yarım gün. 3 tek başına bir oturum. 4 masa başı kararı.

## Bağlam

- Kontrol zinciri denetimi ve geri alınan bulgu: `docs/real_control_chain_audit.md`
- Poz kümesi sonucu ve taban bulgusu: `docs/hand_eye_pose_set_requirement.md`
