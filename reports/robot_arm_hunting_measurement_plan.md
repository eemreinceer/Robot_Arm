# Robot Arm Hunting Joint — Fiziksel Ölçüm ve Kayıt Planı

**Durum:** Ölçüm planı hazır; kök neden çözülmedi.  
**Tarih:** 2026-07-14  
**Kapsam:** Jetson komutu → UART → STM32 parser → PWM → konnektör → güç/toprak → mekanik yük → servo iç kontrolü.  
**Değişiklik kapısı:** İlk temel kayıtlar incelenmeden kod, konfigürasyon, firmware, kalibrasyon, kondansatör, seri direnç veya kablo yönlendirmesi değiştirilmeyecek.

## 1. Kanıt sınırı

### Doğrulanmış gerçekler

- Altı servo daha önce monte halde sessizce tuttu; ROS stack yaklaşık 79 dakika / 237 bin komut hata vermeden çalıştı.
- Kararsızlık multimetre probu ve kablo hareketi sonrasında görüldü; aynı zaman civarında bir `response=''` UART timeout kaydı var.
- Kol son gözlemde rijit fakat kararsızdı; servolar fiziksel olarak aktif karşı koyuyordu.
- Buck terminalinde DMM ile 5.5 V, STM32 GND ile buck OUT- arasında 0.4 ohm + süreklilik ölçüldü.
- `/joint_states` gerçek servo pozisyonu değildir. Encoder yoktur; değer son ACK alınmış PWM komutunun ters eşlenmiş tahminidir.
- Host 50 Hz'e kadar komut gönderir ve çıkışı adım başına en fazla 10 us değiştirir.
- Firmware geçerli `P...` komutunu PWM'e uygular, sonra `OK` gönderir. Protokolde CRC, sequence veya uygulanan değer echo'su yoktur.

### Açık hipotezler

- Aralıklı PWM/VCC/GND konnektörü veya krimp teması.
- Servo konnektöründe kısa süreli ray çökmesi ya da ground-bounce.
- Yüklü, aşınmış, mekanik sınıra dayalı veya iç potansiyometresi sorunlu tek servo.
- Takılı servolardan birinin **continuous rotation** olması. Continuous servo darbe genişliğini konum değil **hız** olarak yorumlar: motor sürülür (zorlayınca dişli direnir → rijit hissedilir) fakat konum hiçbir zaman oturmaz. `servo_calibration.yaml` kaydına göre elbow servosu daha önce tam olarak bu şekilde çıktı ve değiştirildi; parti karışıksa ikinci bir yanlış etiketli parça olabilir. Ölçülmedi, elenmedi.
- Jetson/JTC tarafından değişken hedef üretilmesi.
- Geçerli altı sayı olarak kalan UART bozulması.
- STM32 parser/timer/PWM üretim kusuru.
- Servo motor komütasyonunun PWM/UART hattına kuplajı.

## 2. Çelişkili veya bayat dokümantasyon kaydı

| Kimlik | Konum | Sorun | Geçerli sınır / yapılacak |
| --- | --- | --- | --- |
| DOC-HUNT-01 | Eski `/joint_states` yorumu | `/joint_states` fiziksel feedback gibi yorumlanıyor. | Tek başına yazılım/elektrik ayrımı yapamaz. Controller-state + video ilk kayıt; kesin sınır UART/PWM ölçümüdür. |
| DOC-HUNT-02 | Eski "elenmiş güç sorunu" yorumu | 0.4 ohm, buck terminalinde 5.5 V ve 1 Hz LED kısa transient/yerel bağlantı sorunlarını da elenmiş gösteriyor. | Yalnız kalıcı kopukluk, kalıcı regülatör çökmesi ve kalıcı firmware donması zayıflatılmıştır. Konnektör-local ölçüm gereklidir. |
| DOC-HUNT-03 | Eski "kol gevşek olabilir" yorumu | Daha yeni fiziksel gözlem servoların hunting sırasında karşı koyduğunu söylüyor. | Gevşek/PWM-off çerçevesi bayattır; yine de test günü servo rayı kesilmeden stack durdurulmayacak. |
| DOC-HUNT-04 | Eski servo sınıflandırması | Servoları 360 derece modifiyeli ve eklemleri continuous kabul ediyor. | Tezgah ölçümü pozisyonel 180 derece servo gösterdi; joint_1..5 URDF halen `continuous`, hardware map ise artı/eksi 90 derece clamp ediyor. Uyum ayrı inceleme ister. |
| DOC-HUNT-05 | `servo_calibration.yaml` | Bir yanda `zero_offset_rad=0.0 geçerli`, diğer yanda kanal başına mounted offset ve mekanik limit TODO. | PA0'da boşta bir servo ölçümü tüm monte eklemlerin sıfırını/limitini kanıtlamaz. Değerler ölçülene kadar taslak kabul edilir. |
| DOC-HUNT-06 | `docs/safety/risk_assessment.md` | Eski büyük 6DOF kolun 0.785 m erişim, 1 kg payload ve 30 Nm varsayımlarını kullanıyor. | Robot Arm için güncel fiziksel güvenlik modeli değildir; test günü bu rakamlara dayanılmaz. |

## 3. Test öncesi metadata ve ROS bag komutları

Kayıt Nano üzerinde yerel yapılır; DDS'nin PC'ye çalışması ön koşul değildir.

```bash
mkdir -p /tmp/robot_arm_hunt_capture
date --iso-8601=seconds | tee /tmp/robot_arm_hunt_capture/start_time.txt
git -C /workspace rev-parse HEAD | tee /tmp/robot_arm_hunt_capture/workspace_commit.txt

source /workspace/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 topic list -t | sort | tee /tmp/robot_arm_hunt_capture/topics.txt
ros2 topic info --verbose /joint_states \
  | tee /tmp/robot_arm_hunt_capture/joint_states_info.txt
ros2 topic info --verbose /robot_arm_controller/controller_state \
  | tee /tmp/robot_arm_hunt_capture/arm_controller_state_info.txt
ros2 topic info --verbose /robot_arm_gripper_controller/controller_state \
  | tee /tmp/robot_arm_hunt_capture/gripper_controller_state_info.txt
```

Controller-state topic'leri listede varsa temel kayıt:

```bash
ros2 bag record \
  -o /tmp/robot_arm_hunt_capture/baseline \
  /joint_states \
  /robot_arm_controller/controller_state \
  /robot_arm_gripper_controller/controller_state
```

Topic adı farklıysa tahmin yürütme; `topics.txt` içindeki gerçek `controller_state` adları kullanılır. Kayıt sırasında telefon videosu bütün kolu, özellikle ilk hareket eden eklemi ve mümkünse terminal saatini aynı karede göstermelidir.

Kayıt bitince:

```bash
ros2 bag info /tmp/robot_arm_hunt_capture/baseline \
  | tee /tmp/robot_arm_hunt_capture/bag_info.txt
sha256sum /tmp/robot_arm_hunt_capture/baseline/* \
  | tee /tmp/robot_arm_hunt_capture/sha256sums.txt
```

## 4. Ölçüm sırası ve karar kapıları

### Aşama 0 — güvenlik ve kanıtı koruma

1. Tüm kabloları dokunmadan fotoğraflayıp etiketle.
2. Payload'u çıkar, çalışma alanını boşalt, kolu enerji kesilince çökmeyecek mekanik destekle tut.
3. Fiziksel servo-ray kesiciyi erişilebilir konuma getir.
4. Probları yalnız enerji kapalıyken bağla.
5. Servo rayı açıkken ROS hardware stack'i durdurma; `on_deactivate` → `S` → PWM off kolu düşürebilir.

### Aşama 1 — değişikliksiz temel kayıt

1. Mevcut launch/config ile sistemi aç.
2. ROS bag + video kaydını başlat.
3. Yeni goal gönderme; mevcut sabit durumda hunting oluşmasını bekle.
4. İlk hareket eden eklemi, başlangıç zamanını ve olay süresini not et.

**Karar:**

- Controller desired/reference değişiyorsa Jetson/JTC sınırı önce incelenir.
- Desired/reference sabit fakat fiziksel hareket varsa aşağı katmanlara geçilir.
- `/joint_states` yalnız ACK'li delivered-command zaman çizgisini destekler; gerçek hareket kanıtı değildir.

### Aşama 2 — pasif UART/PWM/rail kaydı

1. Hunting eklemi belirlendikten sonra aşağıdaki kanal planını kur.
2. UART `P...`, STM32 pin PWM'i, konnektör PWM'i ve konnektör-local VCC/GND'yi aynı olayda kaydet.
3. İlk beklenmedik değişimin görüldüğü katman failure boundary'dir.

**Karar zinciri:**

```text
controller reference
  -> Nano TX üzerindeki P alanı
    -> STM32 PWM pini
      -> servo konnektörü PWM + local VCC/GND
        -> fiziksel hareket
```

### Aşama 3 — enerjisiz konnektör testi

Temel kayıt korunduktan sonra servo gücünü kes, kol desteğini doğrula ve hunting ekleminin VCC/GND/PWM iletkenlerini konnektör/kablo hafifçe oynatılırken continuity/min-max ile izle. Değer sıçraması konnektör veya iletken kusurunu doğrular.

### Aşama 4 — yük ve servo iç davranışı

Elektriksel dalga şekilleri sabitse hunting linkini el yerine rijit sling/fixture ile destekleyip aynı sabit komutu tekrar gözle. Hunting yük kalkınca kesilirse mekanik sıkışma, limit veya tork/iç-loop sınırına geçilir. Son adımda şüpheli servo bağımsız servo tester ile sabit PWM altında, kontrollü yükte ve bilinen sağlam servo ile karşılaştırılır.

**Continuous-servo ayırımı (kolu sökmeden, dakikalar sürer):** şüpheli servo koldan çıkarılabiliyorsa boşta, bağımsız servo tester ile sabit nötr darbe (~1500 µs) altında sürülür ve horn izlenir. Belirli bir açıda **durup tutuyorsa** pozisyonel servodur (hipotez elenir); **sabit hızda dönmeye/sürünmeye devam ediyorsa** continuous rotation'dır (hipotez doğrulanır, parça değişir). Ayırıcı gözlem "tutuyor mu" değil, **"oturuyor mu"**dur — continuous servo da zorlamaya direnç gösterir, bu yüzden elle yoklamak yanıltır.

Servo tester yoksa STM32 üzerinden yapılabilir, fakat protokol satırı **altı kanal ister**: tek değer (`P1500`) `E2` döner. Şüpheli kanal sırayla 500/1500/2500 µs'e alınıp diğer beş kanal mevcut değerinde bırakılır (ör. kanal 1 için `P500,<k2>,<k3>,<k4>,<k5>,<k6>`), her adımda ayrı ve **tekrarlanabilir** bir açıda oturup oturmadığına bakılır. Kol monteyse bu adım servo-ray kesici erişilebilirken ve kol desteklenmişken yapılır; 500/2500 µs uç değerleri mekanik çarpışma sınırlarının ölçülmediği eklemlerde riskli olduğundan önce nötr denenir.

## 5. Logic analyzer kanal planı

En az 8 kanal, 3.3 V uyumlu analyzer; örnekleme en az 10 MS/s önerilir. Ground yalnız STM32 logic GND'ye tek noktadan bağlanır.

```text
Logic analyzer

D0 ── PA10  STM32 RX  (Nano TX -> STM32, P... komutları)
D1 ── PA9   STM32 TX  (STM32 -> Nano, OK/E1/E2/E3)
D2 ── PA0   joint_1 PWM / TIM2_CH1
D3 ── PA1   joint_2 PWM / TIM2_CH2
D4 ── PA2   joint_3 PWM / TIM2_CH3
D5 ── PA3   joint_4 PWM / TIM2_CH4
D6 ── PA6   joint_5 PWM / TIM3_CH1
D7 ── PA7   joint_6 PWM / TIM3_CH2
GND ── STM32 logic GND, TEK bağlantı noktası
```

İlk kayıt tüm PWM kanallarından hangi eklemin darbe genişliğinin değiştiğini gösterir. Logic analyzer yalnız dijital zamanlamayı gösterir; genlik, rail dip veya ground-bounce kanıtlamaz.

## 6. Osiloskop kanal planı

Hunting eklemi belirlendikten sonra, problar enerji kapalıyken bağlanır:

```text
4 kanallı osiloskop / mixed-signal scope

CH1 ── STM32 üzerindeki hunting-joint PWM pini - STM32 GND
CH2 ── Servo konnektöründeki PWM sinyali - servo-local GND
CH3 ── Servo konnektöründe VCC - servo-local GND
CH4 ── Servo-local GND - STM32 GND   (differential ölçüm)
TRIG ─ CH1 pulse-width deviation veya CH3 undervoltage
```

Tercih edilen bağlantı:

- CH2/CH3/CH4 differential veya izole prob.
- CH1 için 10x yüksek empedanslı prob ve kısa ground spring.
- Akım probu varsa CH4 yerine hunting servo VCC akımı ölçülebilir.
- Topraklı masa osiloskobunun ground klipsi VCC'ye bağlanmaz.
- Birden çok ground klipsiyle yeni dönüş yolu/ground-loop oluşturulmaz.
- İzole/differential ekipman yoksa CH4 dinamik ölçümü yapılmaz; enerjisiz continuity testiyle sınırlanır.

### Dalga şekli yorumlama

| Bulgular | Sınır |
| --- | --- |
| `P` alanı değişiyor, PWM aynı değişimi izliyor | Jetson/JTC/host command |
| `P` sabit, STM32 PWM pini değişiyor | STM32 parser/timer/firmware |
| STM32 PWM pini sabit, konnektör PWM bozuluyor | PWM kablosu/konnektörü/ground reference |
| PWM sabit, CH3 rail dip/ripple veya CH4 ground shift var | güç dağıtımı/toprak/başka servo etkileşimi |
| UART, PWM, VCC ve GND sabit; fiziksel hareket sürüyor | mekanik yük veya servo iç kontrolü |

## 7. `last_sent` / `last_acknowledged` / `uncertain_state` tasarımı

Bu bölüm yalnız tasarımdır; kodlanmayacak ve UART v1 değiştirilmeyecek.

### Amaç

Host'un "gönderdim", "ACK aldım" ve "fiziksel olarak uygulandı mı bilmiyorum" durumlarını tek `last_pulse_us_` alanında birleştirmesini engellemek.

### Önerilen host durumu

```text
requested_position_rad[6]       JTC'nin son isteği
target_pulse_us[6]              kalibrasyon + clamp sonrası hedef
last_sent_pulse_us[6]           write() ile seri porta tamamı yazılan son frame
last_sent_time                  gönderim zamanı
last_acknowledged_pulse_us[6]   OK ile eşleştirilebilen son frame
last_ack_time                   ACK zamanı
uncertain_state                 uygulanan PWM'in host tarafından kesin bilinmediği durum
uncertain_reason                timeout | E1/E2/E3 | serial_io | activation | stale_ack
ack_timeout_count               monoton sayaç
last_ack_latency_ms             son başarılı ACK gecikmesi
```

### Durum geçişleri

```text
ACTIVATE + V1 handshake
  -> uncertain_state = true          # ilk P/OK görülmedi

full frame write success
  -> last_sent = frame
  -> pending = true
  -> uncertain_state = true          # ACK henüz yok

OK received for the only outstanding frame
  -> last_acknowledged = last_sent
  -> pending = false
  -> uncertain_state = false

E1/E2/E3, timeout or serial I/O error
  -> last_sent korunur
  -> last_acknowledged korunur
  -> uncertain_state = true
  -> sayaç/reason güncellenir
  -> kör retry YOK

DEACTIVATE / S
  -> PWM disabled state ayrıca temsil edilir
  -> joint position fiziksel feedback gibi yayınlanmaz
```

### Tasarım sınırları

- UART v1'de sequence olmadığı için gecikmiş `OK` hangi frame'e ait kesin bilinemez. Aynı anda yalnız bir outstanding frame ve bloklayan request/ACK akışı bu riski sınırlar, tamamen çözmez.
- CRC/sequence/applied-value echo gerçek çözümü güçlendirir fakat UART v1 kontrat değişikliğidir; interface review olmadan yapılmaz.
- `uncertain_state` değeri `/joint_states.position` içine gizlenmez. İleride diagnostic topic/status veya hardware diagnostic interface gerekir.
- Ayrı `serial_response_timeout_ms`, soft-start `step_period_ms`'ten bağımsız planlanır. Değer ölçülen ACK latency dağılımından seçilir; şimdiden sayı uydurulmaz.

## 8. Test günü kısa kontrol listesi

### Başlamadan

- [ ] Kablo/konnektör fotoğrafları ve kanal etiketleri alındı.
- [ ] Payload çıkarıldı, alan boş, kol mekanik destekli.
- [ ] Fiziksel servo-ray kesici erişilebilir.
- [ ] Problar enerji kapalıyken bağlandı; ground-loop riski kontrol edildi.
- [ ] Nano commit, firmware sürümü ve başlangıç saati kaydedildi.
- [ ] Kod/config/kalibrasyon/kablolama değiştirilmedi.

### Temel kayıt

- [ ] Controller-state topic adları keşfedildi.
- [ ] ROS bag Nano üzerinde başladı.
- [ ] Video bütün kolu ve ilk hareket eden eklemi gösteriyor.
- [ ] Hunting başlangıç zamanı ve ilk eklem not edildi.
- [ ] `/joint_states` fiziksel feedback olarak yorumlanmadı.

### Elektriksel kayıt

- [ ] UART RX/TX decode 115200 8N1.
- [ ] Altı PWM kanalı logic analyzer'da görünüyor.
- [ ] Şüpheli PWM STM32 pini ve servo konnektöründe karşılaştırıldı.
- [ ] Servo-local VCC/GND ölçüldü.
- [ ] İlk beklenmedik değişimin katmanı işaretlendi.

### Servo izolasyonu (Aşama 4)

- [ ] Hunting servosu sabit PWM altında **oturuyor mu** diye izlendi (continuous rotation ayırımı, §4 Aşama 4).
- [ ] Sonuç kaydedildi: pozisyonel (hipotez elendi) / continuous (parça değişir) / ölçülemedi.

### Kapatma ve kanıt koruma

- [ ] Önce servo rayı güvenli biçimde kesildi; sonra ROS stack durduruldu.
- [ ] `ros2 bag info` alındı ve dosya hash'leri üretildi.
- [ ] Video, bag, analyzer/scope capture ve fotoğraflar aynı olay kimliğiyle adlandırıldı.
- [ ] Sonuç facts/hypotheses olarak ayrıldı; ölçüm olmadan fix uygulanmadı.

## 9. Kodlama kapısı

Şu dört kanıt görülmeden `arm_hardware`, firmware veya controller config değiştirilmeyecek:

1. Hunting eklemi ve olay zamanı.
2. Controller desired/reference zaman çizgisi.
3. Aynı olayın gerçek UART `P...` alanı ve STM32 PWM pulse-width kaydı.
4. Servo konnektöründe local rail/ground sonucu veya bunların güvenle ölçülemediğinin açık kaydı.

Bu kapı geçilince yalnız kanıtlanan failure boundary için en küçük geri alınabilir değişiklik planlanır.
