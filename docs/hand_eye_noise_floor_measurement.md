# Hand-eye gürültü tabanı ölçüm protokolü

Bu protokol Issue #8'deki 20 pozluk kümenin ölçülen `1.38 mm` zincir
saçılımının hangi bileşenden büyüdüğünü ayırmak içindir. Bir kalibrasyon veya
fiziksel hareket kabulü değildir. Sonuçlar yalnız sonraki deneyi seçer.

## Ölçülen büyüklükler

| Deney | Ölçülen | Referans | Çürütebildiği hipotez |
| --- | --- | --- | --- |
| Aynı hedefe iki yönden tekrar dönüş | Ziyaretler arası `camera→board` konum/dönüş saçılımı ve yön merkezleri farkı | Aynı komutlu hedef, sabit tahta | Saçılım kamera iç-gürültüsüne yakınsa servo tekrarlanabilirliği baskın değildir. Yön merkezleri çakışırsa basit yaklaşma-yönü/backlash açıklaması zayıflar. |
| Dönüş büyüklüğü eğrisi | Tek bir PARK `X` ile kurulan `base→board` kalıntısının bilek dönüş açısı gruplarındaki RMS'i | Kanonik FK q=0 bilek yönelimi | Yüksek dönüş grupları tekrarlı biçimde büyümüyorsa “taban yalnız dönüş büyüklüğünün fonksiyonudur” açıklaması çürür. |

Çerçeveler `base_link`, `link_5`, kamera ve tahta çerçeveleridir. Eklem
birimi rad, dönüş derece, öteleme mm'dir. Kol encoder taşımadığı için
`base→wrist` açık çevrim komut yankısıdır; sonuç mekanizma, zero-offset, FK ve
vizyon terimlerinin toplamını görür.

## Zorunlu güvenlik ve veri kapıları

Canlı koşudan önce hepsi birlikte sağlanır:

1. Plan ve komut satırı bağımsız teknik incelemeden geçmiş olmalı.
2. Kullanıcı robotun başında; çalışma hacmi açık, yerçekimine karşı gereken
   mekanik destek hazır ve fiziksel kesici el altında olmalı. Servo rayı kesik
   başlanır. Kol destekli fiziksel q=0'da görsel olarak doğrulanmalı; yalnız
   `/joint_states` yankısı fiziksel q=0 kanıtı değildir.
3. `scripts/verify_calibration_gate.py` gerçek firmware ile `PASS` vermeli.
4. Issue #13'ün o gün için geçerli fiziksel kapıları ayrıca geçmeli. Bu belge
   onları açmaz veya yerine geçmez.
5. Tahta rijit biçimde kelepçelenmeli. Tamamlanmış, `purpose=calibration`
   q=0 dönüş artifact'i `--gate-from` olarak verilmelidir.
6. Simülasyon çalıştırılmaz. Bu iki araç Gazebo gerektirmez.

`hand_eye_repeatability.py` varsayılan olarak hareket etmez. `--execute` yolu;
Teknik inceleme, kalibrasyon PASS beyanı, gate artifact'i ve tam operatör
teyidi olmadan fail-closed durur. Yaklaşma farkı en çok `0.05 rad`, hız tavanı
en çok `0.10 rad/s` olabilir; mevcut varsayılan `0.05 rad/s` ve `24 s`dir.
SIGINT/SIGTERM ve normal kapanıştaki q=0 dönüş,
tazelik kanıtı ve kesici uyarısı incelenmiş `hand_eye_session.py` tarafından
yönetilir.

## Masa başı hazırlık

Bir hedef için önce yalnız plan üret:

```bash
python3 scripts/hand_eye_repeatability.py \
  --pose-index 0 \
  --plan-out runs/hand_eye/repeatability_pose00_plan.json \
  --raw-out runs/hand_eye/repeatability_pose00_raw.json \
  --report-out runs/hand_eye/repeatability_pose00_report.json
```

Varsayılan altı tekrarın ilk üçü karakterizasyon, son üçü dokunulmadan
validasyondur. Plan ve `.meta.json` dosyası hareketten önce review edilir.
Hedefler ayrı koşulur; böylece hedefler arası büyük geçiş bir tekrar deneyinin
parçası olmaz.

Canlı komut ancak yukarıdaki kapılar gerçekten sağlandığında proje sahibinin
incelediği yollarla tamamlanır:

```bash
python3 scripts/hand_eye_repeatability.py \
  --pose-index 0 \
  --gate-from runs/hand_eye/APPROVED_Q0_GATE.json \
  --architect-reviewed \
  --calibration-gate-pass \
  --known-physical-q0 \
  --operator-confirm KESICI-HAZIR-RAY-KESIK \
  --execute
```

Bu satır bir çalıştırma onayı değildir; dosya adındaki örnek artifact mevcut
olmadıkça araç zaten reddeder.

## Dönüş eğrisi

Kanonik canlı yakalama geldikten sonra analiz-only araç çalıştırılır:

```bash
python3 scripts/hand_eye_rotation_noise.py \
  --samples runs/hand_eye/gated_samples.json \
  --pose-set data/hand_eye/gated_pose_set.json \
  --out runs/hand_eye/rotation_noise_curve.json
```

Araç bütün örneklerden bir kez PARK `X` çözer ve aynı `X`'i her grupta
değerlendirir. Her gruba ayrı `X` uydurmak kötü grubun kendi hatasını gizlerdi.
Üçten az örnekli grup `INCONCLUSIVE` olur. Sentetik analiz kontrolü donanımsız
çalışır:

```bash
python3 scripts/hand_eye_rotation_noise.py --self-test
```

## Önceden ilan edilen yorumlama

- Tekrarlanabilirlik hipotezi, validasyon ziyaret saçılımı aynı koşudaki
  ziyaret-içi vizyon saçılımından belirgin büyükse desteklenir. Pozitif/negatif
  yaklaşma merkezlerinin farkı da aynı kontrolün üzerinde kalıyorsa backlash
  veya yük yönü adayı güçlenir.
- Dönüş hipotezi için en az üç yeterli grup ve en az `45°` kapsama gerekir.
  Yüksek dönüş grupları karakterizasyon ve validasyonda aynı yönde büyümeli;
  tek bir korelasyon katsayısı nedensellik kanıtı sayılmaz.
- Sonuç düz, karışık veya yetersiz grupluysa karar `INCONCLUSIVE` olur; gate
  genişletilmez ve fiziksel GO verilmez.

## Artifact ve geri alma

Her rapor kaynak dosyanın SHA-256'sını, Git commitini, birimleri, planı ve ham
ziyaretleri taşır. `runs/` gitignore altındadır; kabulde kullanılacak en küçük
anonimleştirilmiş fixture ayrıca takip edilen bir yola çıkarılmadan sayı commit
mesajına veya kalıcı gate'e taşınmaz. Bir koşu abort olmuşsa, board check
geçmemişse, `suspect_samples` varsa ya da q=0 dönüşü doğrulanmamışsa veri
kullanılmaz; ray kesilir ve son geçerli artifact'e geri dönülür.
