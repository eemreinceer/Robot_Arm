# Robot Arm elektrik ölçümü analiz araçları

Bu araçlar yalnız daha önce kaydedilmiş dosyaları analiz eder. GPIO, UART, ROS,
servo rayı veya robot hareketine erişmez. `PASS`, fiziksel sistemin bütünü için
değil, yalnız verilen kaydın önceden ilan edilmiş ölçütleri geçtiği anlamına
gelir.

## PWM kaydı

Servo rayı fiziksel olarak OFF ve multimetreyle `0 V` doğrulanmışken GPIO13
(joint_1 kontrol) ile GPIO26 (joint_4 aday) aynı logic-analyzer kaydına alınır:

```bash
sigrok-cli --driver=fx2lafw --config samplerate=1m \
  --channels D0,D1 --samples 200000 -O csv > pwm_q0.csv

analyze_pwm_capture.py pwm_q0.csv \
  --names GPIO13,GPIO26 \
  --expected-us GPIO13=1500,GPIO26=1373
```

Varsayılan kapılar: 50 Hz (`20.0 +/- 0.5 ms`), en az üç tam darbe, pulse
jitter `<=10 us`, beklenen q=0 genişliğine hata `<=20 us`. Düz hat veya kısa
kayıt `FAIL` değil `INCONCLUSIVE` verir; analyzer GND/kanal bağlantısı önce
bilinen bir sinyalle doğrulanmalıdır.

## Servo-local ray kaydı

ESP32 rail-monitor çıktısı dosyaya kaydedildikten sonra:

```bash
analyze_rail_dump.py rail_uart.log --minimum-mv 4800
```

ESP32 ADC mutlak ölçeği multimetreyle aynı sabit anda çapalanır:

```bash
analyze_rail_dump.py rail_uart.log --minimum-mv 4800 \
  --anchor-measured-mv 5520 --anchor-reported-mv 5380
```

Araç `RAILDUMP` bütünlüğünü, ardışık indeksleri, trigger örneğini ve varsayılan
`50 +/- 10 us` zamanlamayı doğrular. Eksik/bozuk veya zamanlaması güvenilmez
kayıt `INCONCLUSIVE`; kalibre edilmiş minimum `4800 mV` altındaysa `FAIL`;
yalnız bütün kapılar sağlanırsa `PASS` döner. JSON çıktı için `--json` kullanılır.

## Ölçüm günü kayıt alanları

Her koşu için firmware commit/hash, kalibrasyon ID, GPIO/servo adı, ham PWM CSV,
ham UART logu, buck ve servo-konnektör DMM değerleri, common-ground ölçümü,
tutma-torku gözlemi, video yolu ve kapanıştaki `ray OFF / PWM OFF` kanıtı birlikte
saklanmalıdır. `/joint_states` gerçek servo konumu değildir.
