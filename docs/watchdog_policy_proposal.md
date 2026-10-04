# Watchdog politikası — öneri ve tezgah planı (D2)

**Tarih:** 2026-08-14. **Karar:** proje sahibinin.
**Durum:** ✅ **C SEÇİLDİ (2026-08-15, kullanıcı) — uygulandı ve tezgahta
sinyalden doğrulandı.** Uygulama `5dfaaad`, firmware `1.4.0-esp32`.
Doğrulama kaydı bu belgenin sonunda.

## Ölçülen durum (iddia değil)

ROS yolunun kullandığı legacy `P` kanalında watchdog
([`firmware/esp32_servo_ctrl/src/main.cpp:545`](../firmware/esp32_servo_ctrl/src/main.cpp)):

```
kLegacyWatchdogMs = 1000
→ süre dolunca: linkWrite("E3\n")   ve BAŞKA HİÇBİR ŞEY
```

PWM'e dokunulmuyor, nötre alınmıyor, disarm edilmiyor. Smooth yolun timeout'u
(`kCommunicationTimeoutMs = 2000`) ise `STATE=DISARMED, PWM=HOLD` diyor — o da
darbeyi kesmiyor, sadece durumu değiştiriyor.

Üstelik `E3` satırını **ROS tarafında okuyan kimse yok** (`arm_hardware`
kaynağında `E3` geçmiyor). Asenkron geldiğinde bir sonraki ACK okumasına
karışabilir ve "MCU reddetti" gibi görünür — kazara fail-closed, tasarımla değil.

**Sonuç:** Jetson ölürse, kablo çıkarsa ya da stack donarsa **servolar son
komutu süresiz tutar.** Kol havada kalır, tork devam eder, kimse bilmez.

## Üç seçenek

### A) TUT (mevcut davranış, ama bilinçli seçilmiş)

Son PWM latch'li kalır. `E3` basılır.

- **Lehine:** Kolun freni yok. PWM kesilirse kol **düşer**; havada bir yükle ya
  da tahtanın üstünde duruyorsa çarpar. Tutmak, en az hasarlı ani davranıştır.
- **Aleyhine:** Süresiz tork, hobi servolarında ısınma ve ömür kaybı demek.
  Ayrıca "sistem öldü ama kol hâlâ enerjili" durumu operatöre görünmez.

### B) KADEMELİ NÖTRE AL

Timeout sonrası PWM'i soft-start rampasıyla (10 µs/adım) güvenli bir duruşa
indir, sonra kes.

- **Lehine:** Tork süresiz kalmaz.
- **Aleyhine:** "Güvenli duruş" bu kolda **tanımlı değil** — encoder yok, kolun
  nerede olduğu bilinmiyor, ve q=0 her pozdan güvenli değil (nesneye ya da
  masaya çarpabilir). Tanımsız bir hedefe otonom hareket, watchdog'un çözmesi
  gereken sorundan daha büyük bir sorun üretir.
- **Değerlendirme:** encoder gelmeden bu seçenek savunulamaz.

### C) TUT + DUYUR (önerilen)

Davranış A ile aynı: PWM latch'li kalır. Fark, olayın **görünür** olması:

1. Firmware `E3`'ü **tekrarlar** (ör. saniyede bir), tek sefer değil — çünkü tek
   satır, karışan bir ACK içinde kaybolabilir.
2. `arm_hardware` bu satırı tanır ve ROS tarafında **hata olarak** raporlar:
   log + `on_error` yolu → stop frame + disarm. Yani host **ayaktaysa** kol
   kontrollü biçimde bırakılır; host ölmüşse firmware tutmaya devam eder.
3. Operatör için fiziksel gösterge: heartbeat LED'i timeout'ta desen değiştirir
   (şu an 500 ms sabit yanıp sönüyor, `updateHeartbeat`).

- **Lehine:** Frensiz kol için doğru olan "tut" davranışını korur, ama sessizliği
  kaldırır. Host ayaktayken zaten daha iyi bir yol var (kontrollü disarm), ve o
  yol şu an kullanılmıyor.
- **Aleyhine:** `E3`'ü tekrarlamak seri hattı meşgul eder; ACK penceresiyle
  çakışmaması için sıklık ölçülmeli.

## Öneri

**C.** Gerekçe: A'nın fiziksel davranışı bu kol için doğru, ama şu an *seçilmiş*
değil — miras. C, aynı fiziksel davranışı korurken üç şeyi ekler: tekrar eden
duyuru, host tarafında tanıma, ve operatöre görünür gösterge. B, encoder
gelmeden savunulamaz ve encoder geldiğinde yeniden değerlendirilmeli.

## Tezgah doğrulama planı (karar verildikten SONRA, RAY KESİK)

Değişiklik canlı kolda değil, tezgahta doğrulanır. Kol raydan ayrık; tek servo
ya da hiç servo yok, ölçüm **sinyalden** okunur, log'dan değil.

1. **Referans:** ESP32'yi besle, `P` çerçevesi gönder, PWM'i osiloskop/logic
   analyzer ile gör. Bu, "PWM var" durumunun kanıtı.
2. **Timeout:** komut akışını kes, kronometre. Beklenen: 1000 ms sonra `E3`
   (C'de tekrarlı), **PWM değişmeden devam ediyor**. Ölçülen: PWM darbesi hâlâ
   var mı, genişliği aynı mı.
3. **Host tanıma:** `arm_hardware` ayakta ve `E3` geliyorken, bileşenin ERROR'a
   geçtiği ve stop frame gönderdiği doğrulanır (`test_error_transition` bu yolu
   zaten kapsıyor; buradaki yeni kısım tetiğin `E3` olması).
4. **Regresyon:** `scripts/verify_stop_frame.py` mevcut ve tam bu tür bir kanıt
   için yazılmış — `STATE=DISARMED` görmenin PWM kesildiği anlamına
   GELMEDİĞİNİ orada not düşmüşüz. Aynı disiplin burada da geçerli: **PWM'i
   açıkça oku.**

**Kabul:** üç ölçümün üçü de kayda geçmeden değişiklik canlı kola gitmez.

## Ne yapılmadı

Firmware'e tek satır yazılmadı. Bu bilinçli: değişiklik güvenlik davranışını
değiştiriyor ve doğrulaması fiziksel. Donanımsız yazılmış, denenmemiş bir
güvenlik yaması, daha önce işaretlenen test kusurunun aynısı olurdu —
koşmamış bir şeyin çalıştığını varsaymak.

---

## Tezgah doğrulaması — 2026-08-15, RAY KESİK, kol hareket etmedi

Firmware `1.4.0-esp32`, ölçüm Saleae/fx2lafw logic analyzer ile 1 MHz'de,
9 saniyelik tek yakalama. Nano `/dev/ttyTHS1` üzerinden 20 Hz'de 3 saniye
`P1500×6` akıttı, sonra göndermeyi kesti.

| # | Ölçüm | Beklenen | Ölçülen | Sonuç |
| --- | --- | --- | --- | --- |
| 1 | Referans PWM (GPIO13) | 50 Hz, 1500 µs | 49.99 Hz, **1500.4 µs** (min 1500, max 1501), periyot 20.006 ms | PASS |
| 2 | Timeout'ta PWM | değişmeden sürer | akış kesildikten sonra 5 saniye boyunca her saniye **50 darbe, aynı 1500.4 µs** | PASS |
| 3 | `E3` duyurusu | 1000 ms sonra, tekrarlı | ilk `E3` **0.956 s**, tekrar aralığı **1.000 s** (min 0.997, max 1.007), 5 kez | PASS |
| 4 | LED alarmı (GPIO2) | 500 ms → 100 ms | `500,500,…` → 196 (geçiş) → `100,100,100,…` | PASS |
| — | ACK bütünlüğü | hepsi | **60/60 OK** | PASS |

4'teki 196 ms'lik tek ara, bayrak kalktığında LED'in periyodunun ortasında
olmasındandır; bir sonraki kenardan itibaren desen 100 ms'ye oturuyor.

**Böylece politika C'nin fiziksel iddiası kanıtlandı:** host sustuğunda MCU
darbeyi kesmiyor, genişliğini de değiştirmiyor — tork son komutta kalıyor — ve
bunu artık hem seri hatta (tekrarlı `E3`) hem de göz ile (100 ms LED) duyuruyor.

### Bu ölçüme varmak dört bozuk denemeyi aldı — sebepleri kayda değer

Rakamlardan daha öğretici olan kısım bu. Aynı ölçüm dört kez alındı ve dördü de
farklı bir sebeple geçersizdi; hiçbiri firmware hakkında bir şey söylemiyordu:

1. **UART harness'ı sökülüydü.** 60 çerçeve gönderildi, hiç ACK gelmedi, `E3`
   gelmedi, PWM görünmedi. Üçü de "watchdog çalışmıyor" gibi okunabilirdi.
   Araç ACK sayısını basmadığı için ayrım yapılamıyordu — düzeltildi (`646f6ae`).
2. **Firmware satır tamponunda çöp vardı.** Sökük RX hattı gürültü toplamış,
   ilk gerçek istek `ERR,LINE_TOO_LONG` döndü. Araç artık açılışta tamponu
   boşaltıyor.
3. **Analyzer toprağı servo konnektörünün GND'sindeydi**, ESP32'nin GND'sinde
   değil. Ray kapalı olduğu için referans yüzüyordu; her iki kanal da **50.01 Hz**
   şebeke gürültüsü topluyordu. Bu, `measure_pwm_capture.py`'nin başındaki
   uyarının ve README'deki "ortak sinyal-referans GND" kuralının aynen ihlaliydi.
4. **Kart beslemesizdi.** GND düzeltilirken USB çıkmış; `lsusb`'de CP2102 yok,
   her iki GPIO düz LOW.

Bu dizinin ortasında bir ara **"GPIO13 sessiz, firmware PWM açtığını sanıyor"**
sonucuna varılacaktı — `CHANNEL_3` teşhisinin ESP32 karşılığı gibi duruyordu ve
**yanlış** olacaktı. Onu durduran tek şey, kontrol kanalının olmamasıydı:
sabit LOW, "prob sağlam, pin sessiz" ile "prob ölü" arasında ayrım yapmaz.

**Kural, ölçümden önce kontrol kanalı kurmaktır.** Burada kontrol kanalı
heartbeat LED'iydi (GPIO2): doğru toprakla 500.1 ms'lik temiz kare dalga
görülünce zincirin tamamı — prob, kanal, toprak, analyzer — tek seferde
kanıtlandı. O kanıt gelmeden alınan hiçbir "sinyal yok" okuması kanıt değildir.

### Host tanıma — ✅ kapandı (2026-08-15, aynı gün, donanımsız)

Öneri listesindeki 3. madde de doğrulandı: `scripts/fake_esp32.py` artık
`--fault-mode watchdog-e3` ile `OK` yerine `E3` cevaplıyor — firmware'in
watchdog'unun hatta bastığı şeyin aynısı — ve `scripts/verify_error_path.sh`
her iki arızayı da gerçek `controller_manager` ile koşuyor
(`verify_error_path.sh [silence|watchdog-e3|both]`).

| Arıza | Host'un gördüğü | Stop frame MCU'ya vardı mı |
| --- | --- | --- |
| `silence` | ACK 20 ms'de timeout | ✅ arızadan **32.1 ms** sonra |
| `watchdog-e3` | tamamlanmış ama `OK` olmayan cevap | ✅ arızadan **7.6 ms** sonra |

Aradaki fark beklenen yönde: `E3` bir cevap olarak geldiği için host timeout
penceresini beklemiyor. Kanıt MCU tarafından alınıyor — `S` çerçevesinin
gerçekten tele düştüğü sahte ESP32'nin raporundan okunuyor, host log'undan
değil.

Log da artık doğruyu söylüyor. Eskiden bu satır "ESP32 rejected command,
response='E3'" idi; olanın tersi. Şimdi watchdog'u adıyla anıyor ve torkun
basılı kaldığını, ardından gelen hata geçişinin PWM'i keseceğini söylüyor —
yani operatöre kolu desteklemesi gerektiğini.

**Böylece politika C'nin üç parçası da doğrulandı:** tekrarlı duyuru ve latch'li
PWM (sinyalden), LED alarmı (sinyalden), host tanıma ve kontrollü disarm
(canlı ROS yığınıyla). Bu belgede açık madde kalmadı.
