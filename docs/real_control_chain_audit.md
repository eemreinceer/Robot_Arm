# Gerçek kol kontrol zinciri — denetim (2026-08-14)

**Kapsam:** `main` üzerindeki gerçek donanım yolu, MoveIt'ten servo PWM'ine ve
geri `/joint_states`'e. Denetim sırasında bulunan iki kusur aynı gün düzeltildi;
bir bulgu ise **yanlış çıktı ve geri alındı** — aşağıda ikisi de duruyor, çünkü
bu depoda iddia ile ölçüm arasındaki farkın kaydı tutuluyor.

## Komut yolu (ölçülen sıra)

| # | aşama | dönüşüm | yer |
| --- | --- | --- | --- |
| 1 | `active_` kapısı | — | `write()` başı |
| 2 | armed kapısı | — | değilse OK, hiçbir şey gönderilmez |
| 3 | arming latch'i | rad | referanstan `last_pulse_us_` + `commanded` tohumlanır |
| 4 | komut hızı kapısı | — | `command_rate_hz_` (50 Hz) |
| 5 | güvenlik zarfı | rad → rad | `clamp_into_safe_range` (reddetmez, katlar) |
| 6 | hız limiti | rad → rad | `ActivationSafety::filter` |
| 7 | kalibrasyon | rad → µs | `position_to_microseconds` (+offset, clamp, ölçek, invert) |
| 8 | soft-start slew | µs → µs | `±max_delta_us_per_step` (10 µs) |
| 9 | kanal haritalama | eklem → kanal | `calibration_[i].channel - 1` |
| 10 | seri | µs → ASCII | `P<us>,…\n` + ACK |

**İki hız sınırı vardır ve aynı şey değildirler:** radyan tarafındaki fiziksel
hız limiti (6) ve µs tarafındaki soft-start (8). Biri "kol ne kadar hızlı
dönebilir", diğeri "PWM ne kadar hızlı değişebilir".

**Limitler kalibrasyondan ÖNCE uygulanır** (5–6 → 7). Doğrusu budur: limitler
radyan uzayında tanımlıdır ve `invert: true` olan bir eklemde µs tarafında clamp
etmek min/max'ı ters çevirirdi.

## Geri besleme yolu

`read()` → `position_states_` / `velocity_states_` → `joint_state_broadcaster`
→ `/joint_states` → RSP / TF / MoveIt.

`commanded/*` GPIO'su bu zincire **bağlı değildir** ve bağlanmamalıdır; hiçbir
controller onu talep etmez. Salt gözlemlenebilirlik altyapısıdır.

## Bulgular

### 1. `commanded`, arming anında tohumlanmıyordu — DÜZELTİLDİ (`f467e43`)

0.10 rad referansta arming'den sonra `position` 0.1005 iken `commanded` 0.0
okuyordu: ayrımı gözlemlenebilir kılmak için eklenen kanal, **hiç yaşanmamış**
0.10 rad'lık bir izleme hatası bildiriyordu.

Testi yazarken çıkan yan ders: peer üzerinden `arm()` çağırmak
`arm_initialization_pending_` latch'ini atlıyor (o latch `on_safety_parameters`
içinde, disarmed→armed kenarında set ediliyor). Gerçek arming kenarı
modellenmeden kusur görünmüyordu.

### 2. Kalibrasyon damgası üretiliyor ama doğrulanmıyordu — DÜZELTİLDİ (`fd26a37`)

`kCalibrationSha256` başından beri header'da duruyordu ve kimse bakmıyordu.
Üretecin SHA'sı karşılaştırılamazdı: normalize edilmiş sözlükler üzerinden
alınıyor ve `FIRMWARE_POLICY`'deki `pin`/`ledc`/`deadband` alanlarını içeriyor —
bunlar ROS tarafına hiç ulaşmıyor.

Bunun yerine **iki tarafın paylaştığı anlambilim** parmak izleniyor: kanal
haritası, ölçek çapaları, güvenlik limitleri, sıfır ofseti, pulse aralığı, yön.
Mikro-radyana kuantalanıp FNV-1a 64 ile hash'leniyor; firmware `K?`'ye üretecin
bastığı sabitle cevap veriyor.

Uyuşmazlık ile doğrulayamama **ayrı tutuldu**: farklı sayı bildiren firmware her
koşulda reddedilir (PWM üretilmeden); `K?` bilmeyen eski firmware uyarı alır ve
`require_calibration_match:=true` ile o da redde çevrilir.

### 3. ⚠ GERİ ALINAN BULGU: "firmware limitleri ROS'tan dar"

Denetimin ilk turunda firmware'in joint_3/joint_4 limitlerinin ROS'tan dar
olduğu ve ROS'un bunu bilmediği yazıldı. **Yanlış.** Sayılar çaprazlandığında
ikisinin aynı limit olduğu görüldü; firmware onları sıfır ofseti çıkarılmış
eklem koordinatında saklıyor:

```
joint_3: limit ±1.52 rad = ±87.090°,  zero_offset 1.2403 rad = 71.064°
         -87.090 − 71.064 = -158.154°   ✓ firmware
          87.090 − 71.064 =   16.026°   ✓ firmware
```

Düzeltilecek bir şey yoktu. Kayda geçiyor çünkü bu bulguya göre iş planlanmıştı.

### 4. `stopped_velocity_tolerance` etkisiz

`read()` `velocity_states_`i her döngüde 0.0 dolduruyor, dolayısıyla JTC'nin
"hedefte durdu mu" kapısı her koşulda geçiyor. Değer silinmedi, encoder geldiği
gün anlam kazanacağı için yorumla işaretlendi (`robot_arm_controllers_real.yaml`).

### 5. Legacy watchdog PWM'e dokunmuyor — AÇIK, fiziksel doğrulama bekliyor

ROS yolunun kullandığı `P` kanalında watchdog (`main.cpp`, 1000 ms) yalnız bir
kez `E3\n` yazıyor; PWM'i kesmiyor, nötre almıyor. Smooth yolun timeout'u da
`PWM=HOLD` diyor. Frensiz bir kolda "tut" savunulabilir bir varsayılandır — ama
sonuç şudur: **Jetson ölürse tork süresiz kalır** ve `E3` satırını ROS tarafında
okuyan kimse yok.

Bu bilinçli bir politika hâline getirilmeli. Değişiklik firmware davranışını
değiştirdiği için ray kesikken, operatör başında doğrulanmalı → fiziksel erişim
gerekiyor, bugün kapatılmadı.

## Test kapsamı (bu denetimden sonra)

| konu | durum |
| --- | --- |
| rad↔PWM gidiş-dönüş | ✅ `test_calibration_mapping` |
| yön çevirme (`invert`) | ✅ eklendi — daha önce hiç test edilmemişti |
| sıfır ofseti ölçeği bozmuyor | ✅ eklendi |
| kanal haritalama (kanal ≠ eklem sırası) | ✅ eklendi |
| komut/ölçüm ayrımı | ✅ `test_commanded_state` |
| kalibrasyon parmak izi + kapı | ✅ `test_commanded_state` |
| eklem limitleri, hız limiti | ✅ `test_activation_safety` |
| ACK / timeout / bozuk çerçeve / drain | ✅ `test_serial_ack` |
| lifecycle hata geçişi + stop frame | ✅ `test_error_transition` (Jazzy'de) |
| **firmware watchdog** | ❌ host tarafından görünmüyor |
| **`K?` alışverişi gerçek donanımda** | ❌ flash gerekiyor |

Testlerin hepsi CMakeLists'te kayıtlı ve `colcon test` ile koşuyor. Kayıtsız
test dosyası bırakma hatası (2026-08-14, `6c20250`) bu yüzden ayrıca izleniyor.

## Fiziksel erişim gerektiren, açık kalanlar

1. ESP32'yi flash'la → `K?` yolu canlı doğrulansın → `require_calibration_match:=true`.
2. Legacy watchdog politikası (tut/nötre al/duyur) → ray kesikken doğrula.
3. Encoder montajı → `read()` sensörden doldurulur, `position_is_measured_` 1.0 olur.
