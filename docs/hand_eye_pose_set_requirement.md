# Hand-eye poz kümesi şartı

**Durum:** AKTİF şart. Gerçek kolda `--purpose calibration` koşusu yapılmadan
önce sağlanmalıdır.
**Kaynak:** sim provası ölçümleri, issue #8. Prova kanonik başlatıcıyla
koşuldu — `./start_simulation.sh --hand-eye --headless` (2026-08-12).
Buradaki sayılar sim gerektirmeden tekrar üretilir:
`scripts/hand_eye_convention_check.py`, `scripts/hand_eye_noise_sweep.py`;
provanın ulaşılabilir poz kümesi `data/hand_eye/sim_wrist_poses.json` olarak
takiptedir (üreten: `scripts/hand_eye_pose_set_export.py`).

## Neden bu belge var

Sim provasında `camera_mount` transformu **bilinerek** verildi ve çözücü onu
~9 mm hatayla buldu. Elenenler ölçümle elendi:

| aday | nasıl elendi |
| --- | --- |
| konvansiyon uyuşmazlığı | sentetik kusursuz veride beş yöntem de X'i tam verdi |
| zincir (FK/TF/PnP) | yer gerçeği X ile sabit tahta 0.41 mm rms / 0.68 mm maks'a kuruluyor |
| çözücü hatası | aynı çözücü, iyi pozlarla 0.4 mm gürültüde 0.55 mm veriyor |

Kalan sebep **poz geometrisi**. Aynı 0.4 mm gürültü, kolun tahtayı kadrajda
tutarken ulaşabildiği pozlarda **4.56 mm**'ye büyüyor — çeşitli pozlarda 0.55 mm.
Sekiz kat fark, tek değişken poz kümesi.

**Gerçek kolda bu hata görünmez.** Karşılaştırılacak yer gerçeği yoktur ve
çözücünün başlık metriği yakalamaz: kanonik provada çözülen X yer gerçeğinden
**8.9 mm** sapmışken tahta saçılımı yalnız **0.98 mm** okuyor (doğru X ile aynı
veri: 0.41 mm). Mutlak olarak makul bir sayı.

## Şart

Testler ve uygulama aynı ilan edilmiş kapıyı kullanır. Makinece okunabilir
kanonik değerler şunlardır; bunlardan biri değişirse gerekçe ve yeni fiziksel
ölçüm kanıtı aynı değişiklikte verilmelidir:

| sabit | değer |
| --- | ---: |
| `TARGET_MM` | `1.0` |
| `NOISE_FLOOR_MM` | `0.41` |
| `K_GATE` | `2.4390243902439024` |

### 1. Hedef doğruluk KOŞUDAN ÖNCE yazılır

`X` için kabul edilebilir öteleme ve dönme hatası, koşudan önce yazılı olarak
ilan edilir. Sonradan ölçülen değere göre türetilmez.

Bu, issue #4'te zaten uygulanan kuralın aynısıdır: eşik önce ilan edilir, yoksa
mevcut sapmayı örtecek şekilde seçilme riski vardır.

### 2. Gürültü tabanı ölçülür

Koşunun kendi verisinden, yer gerçeği olmadan hesaplanabilen tek tutarlılık
ölçüsü kullanılır: sabit tahtanın `T_base_board = A·X·B` ile yeniden kurulum
saçılımı. Bu, gürültü tabanının **alt sınırıdır**.

### 3. Duyarlılık ölçülür — asıl kapı budur

Aday poz kümesinin `A` matrisleri alınır, bilinen bir `X` ve sabit tahtadan
kusursuz `B` üretilir, üzerine **ölçülen gürültü tabanı** enjekte edilir ve
çözücünün hatası ölçülür. Büyütme katsayısı:

    k = |X_çözülen − X_gerçek| / enjekte_edilen_gürültü

**Kabul kriteri:** `k(p90) × (ölçülen gürültü tabanı) ≤ (1. maddede ilan edilen hedef)`

### ⚠ k TEK ÇEKİLİŞTEN OKUNMAZ

Bu belgenin ilk sürümü `k ≈ 11` yazıyordu ve bu **tek bir gürültü çekilişinin**
değeriydi. Kötü koşullu bir kümede hata, gürültünün hangi yöne düştüğüne
kuvvetle bağlıdır; aynı küme 300 çekilişte şunu veriyor:

| poz kümesi | ortalama | medyan | p90 | maks |
| --- | --- | --- | --- | --- |
| provanın kümesi (5 poz) | 3.85 | 3.50 | **6.41** | 13.73 |
| çeşitli sentetik (8 poz) | 1.44 | 1.36 | **2.34** | 3.42 |

Tek çekiliş 11.4 de gösterebilir, 1.2 de — ikisi de kümeyi tarif etmez. Kapı bu
yüzden **p90** üzerinden kurulur: kötü ama makul bir çekilişte de hedef tutmalı.
Ölçüm `scripts/hand_eye_pose_search.py` içinde, arama sırasında ucuz vekil
(ortalama), kapıda 300 çekilişlik p90.

`hand_eye_noise_sweep.py`'ın tablosu da artık çok çekilişin medyanı ve p90'ı
olarak basılıyor; tek çekiliş basan sürümü bu hatayı üretiyordu.

5. maddedeki yasağın sayısal hâli aynı tabloda: sim kümesi ~9 mm yanlışken tahta
saçılımı ~1.3 mm okuyor, yani provada gözlenen körlük sentetik olarak da
üretiliyor.

Poz kümesi bu kapıdan geçmiyorsa **koşu yapılmaz** — poz kümesi değiştirilir.
Bu kapı hareketsizdir ve robot gerektirmez; koşudan önce masa başında geçilir.

### 4. Kadraj verimi hesaba katılır

Kanonik provada 18 aday pozdan yalnız **5'i (%28)** tahtayı tespit edilebilir
tuttu (`data/hand_eye/sim_wrist_poses.json` içinde kayıtlı). Daha eski ve
depoda bulunmayan 57 pozluk tarama kümesi %23 vermişti — aynı mertebe, ama o
küme yeniden üretilemiyor, bu yüzden şart %28'lik ölçülen değere dayanır.

Aday havuzu, hedeflenen örnek sayısının en az **4 katı** olacak şekilde kurulur;
aksi hâlde koşu ortasında örnek sayısı yetersiz kalır.

Kadrajda kalma, poz kümesi kabul edilmeden önce **sim'de** doğrulanır.

### 5. Yasaklar

- **Tahta saçılımı tek başına kabul kapısı yapılamaz.** 8.9 mm yanlış bir X'te
  0.98 mm okuyor. Duyarlılık kapısını (3. madde) geçmeden anlamlı değildir.
- **Yöntemler arası yayılım koşullanma vekili değildir.** Bu belgenin yazarı
  bir kez öyle sayıp "koşullanma elendi" sonucuna vardı; yayılım 17 kat
  oynarken hata sabit kaldı. Yöntemlerin uyuşması, tahminin gürültüye
  dayanıklılığı hakkında bilgi vermez.
- Koşu sonrası hedefin gevşetilmesi yasaktır.

## Ön koşul — şu an sağlanmıyor

Bu şart poz kümesi hakkındadır ve **issue #4'ün yerine geçmez.** Kol aynı q=0'a
fiziksel olarak 8–19 mm tutarsızlıkla dönüyor ve 2026-08-05'te altı servoda
tutma torku bulunmadı. Dönüş tekrarlanabilirliği çözülmeden hiçbir poz kümesi
işe yaramaz: yukarıdaki gürültü tabanı, kolun kendisi tekrarlanamazken
ölçülemez.

Sıra: elektriksel katman → dönüş tekrarlanabilirliği (#4) → bu şart →
`--purpose calibration`.

---

## 2026-08-14 ARAMA SONUCU — kapı sayısal olarak geçti, GERÇEK VERİ yalanladı

Sim'de 400 aday poz sürüldü, **399'u** tahtayı kadrajda tuttu (kör rastgele
havuzda bu oran %4'tü; fark FK ile kadraj kestirimi). 399'luk havuzdan k'yı en
küçük yapan **20 pozluk** alt küme arandı.

**Sentetik kapı (0.41 mm taban ile):**

```
k ortalama 1.03 | medyan 0.92 | p90 1.81 | maks 2.58
p90 × 0.41 = 0.74 mm  (hedef 1.0 mm)  ->  GEÇTİ
```

**Aynı kümenin GERÇEK yakalamasıyla çözümü:**

| yöntem | t (mm) | yer gerçeğinden |
| --- | --- | --- |
| PARK | [22.65, −0.12, 59.06] | **2.81 mm** |
| TSAI | [22.56, 0.00, 59.05] | 2.73 mm |
| HORAUD | [22.65, −0.10, 59.05] | 2.82 mm |
| DANIILIDIS | [22.72, −0.24, 58.60] | 3.07 mm |

Provanın 8.9 mm'sine göre **üç kat iyileşme**, ama hedef 1.0 mm ve **tutmuyor.**

### Kök neden: gürültü tabanı sabit değil, poz çeşitliliğiyle büyüyor

Kapı 0.41 mm'lik bir taban varsayıyordu; o değer **5 pozluk provadan** ölçülmüştü.
Aynı ölçüm bu 20 pozluk kümede **1.38 mm rms** veriyor — 3.4 kat. Sebep fiziksel:
geniş bilek dönüşleri FK / zero-offset / servo hatasını büyütür, ve poz
çeşitliliği tam da aramanın **artırdığı** şeydir.

Model yanlış değildi, **girdisi** yanlıştı:

```
k(p90) 1.81 × gerçek taban 1.38 mm = 2.50 mm   ≈   ölçülen 2.81 mm
```

Ölçülen tabanla kapı yeniden kurulduğunda sonuç **KALDI** (2.49 mm > 1.0 mm) —
`data/hand_eye/gated_pose_set.json` bu hâliyle kayıtlı.

### Şarta eklenen madde

**Gürültü tabanı, kapıya sokulan poz kümesinin KENDİ yakalamasından ölçülmelidir.**
Başka bir kümeden devralınan taban, geniş kümelerde sistematik olarak iyimserdir.
`hand_eye_pose_search.py select --noise-floor <mm>` bu yüzden var; varsayılanı
kullanmak uyarı bastırır ve artifact'e `"noise_floor_source": "provadan
devralindi (SUPHELI)"` yazar.

### Bundan sonrası poz geometrisi işi DEĞİL

20 pozluk küme, havuzun tamamının k'sına (0.31) yakın; poz seçiminden alınacak
kazanç bitmiştir. Kalan 2.8 mm, **tabanın kendisidir** — yani FK doğruluğu,
zero_offset ve servo tekrarlanabilirliği. Hedefe gitmenin yolu daha iyi poz
aramak değil, tabanı küçültmektir; o da fiziksel iştir (#4 dönüş
tekrarlanabilirliği, servo tutma torku).

---

## ⚠ 2026-08-14 GEÇ SAAT — YUKARIDAKİ AÇIKLAMA YANLIŞTI, DÜZELTİLDİ

Birkaç saat önce bu belgeye "gürültü tabanı poz çeşitliliğiyle büyüyor (0.41 →
1.38 mm), sebebi geniş bilek dönüşlerinin FK/zero-offset/servo hatasını
büyütmesi" diye yazdım. **İki yerden birden yanlış.**

### Yanlış 1 — karşılaştırılan iki sayı aynı şeyi ölçmüyordu

1.38 mm değeri `solve_hand_eye.py`'ın `board_consistency` çıktısıydı ve o
fonksiyon tahtayı **ÇÖZÜLEN X ile** yeniden kuruyor (`solve_hand_eye.py:85`).
Çözüm 2.81 mm yanlışken saçılım da şişiyor. Yani çözümün hatasını, o hatayla
kirlenmiş bir sayıyla yargılamışım — **döngüsel**.

Yer gerçeği X ile aynı örnekler:

| küme | öteleme sapması (medyan) | açısal sapma (p90) |
| --- | --- | --- |
| prova, 5 poz | 0.37 mm | 0.26° |
| aranmış, 20 poz | 0.47 mm | 0.46° |

Taban 3.4 kat büyümedi. Neredeyse aynı kaldı.

### Yanlış 2 — baskın gürültü kanalı öteleme değil, DÖNME

Kapı yalnız tahtanın **öteleme** hatasını enjekte ediyordu. Ölçüm (20 pozluk
küme, 200–300 çekiliş):

| enjekte edilen | X hatası (ort) |
| --- | --- |
| 0.41 mm öteleme, 0° dönme | **0.45 mm** |
| 0.41 mm öteleme, 0.5° dönme | **2.94 mm** |
| 0 mm öteleme, 1.5° dönme | 8.73 mm |
| 0.41 mm öteleme, 1.5° dönme | 8.74 mm |

Son iki satır kararı veriyor: **öteleme gürültüsünün katkısı, dönme yanında
ölçülemeyecek kadar küçük.** Gerçek veride ölçülen 2.81 mm, ölçülen açısal
tutarsızlıkla (p90 0.46°) birebir uyuşuyor — düzeltilmiş model 1.72–2.69 mm
veriyor, eski model 0.49 mm veriyordu.

### Şarta işlenen düzeltme

1. **Taban AÇISAL birimde ilan edilir.** Tahtanın oryantasyon tutarlılığı
   (derece), öteleme saçılımı değil. "k × mm taban" çarpımı anlamını yitirdi:
   `k` yalnız öteleme gürültüsüne göre tanımlıydı.
2. **Taban, çözülen X ile hesaplanan bir sayıdan okunamaz.** `board_consistency`
   bu iş için kullanılamaz; yer gerçeği olan sim'de doğrudan ölçülür, gerçek
   kolda ise bağımsız bir referans gerekir.
3. `hand_eye_pose_search.py` artık dönme gürültüsü de enjekte ediyor
   (`k_samples(..., sigma_deg=)`) ve `expected_error_mm()` hatayı doğrudan
   veriyor.

### Bunun sıradaki işe etkisi

Hedefe gitmek için **tahtanın oryantasyon hatasını** küçültmek gerekiyor: tahta
tasarımı (ChArUco, daha çok köşe), görüş mesafesi ve açısı, çözünürlük. Bu
öncelikli olarak bir **algı** işidir — servo tekrarlanabilirliği ve mekanik
tarafın buradaki payı henüz ölçülmedi ve sim verisi onlar hakkında hiçbir şey
söyleyemez (sim'de ikisi de kusursuz).

Bugün 399 pozluk havuzda ayrıca ölçülen ikinci bir şey: **örneklerin ~%14'ü
düzlemsel PnP çift-çözüm belirsizliğine düşüyor** (100–215 mm sapma, üstelik
reprojeksiyonları DÜŞÜK — yanlış çözüm görüntüye neredeyse aynı iyi oturuyor).
Bunlar neredeyse cepheden bakılan pozlarda yoğunlaşıyor. Aranmış 20'likte flip
yok, ama bu şansa bırakılamaz: poz üretimi cepheye çok yakın bakışlardan
kaçınmalı ve/veya ChArUco'ya geçilmeli. Ölçüm aracı:
`scripts/hand_eye_floor_analysis.py`.


---

## Issue #8 sabit sapması — kaynak daraltıldı (2026-08-15, çevrimdışı)

Açık soru şuydu: 8.92 / 9.34 / 9.07 mm'lik sapma **örnek kurma konvansiyonundan
mı çözücüden mi** geliyor. Sentetik veriyle ayrıştırıldı; sim gerekmedi.
Araç: `scripts/hand_eye_bias_decomposition.py`.

**Çözücü aklandı.** `solve_hand_eye.py --self-test` kusursuz veride beş
yöntemin beşiyle de X'i **0.0000 mm** hatayla geri kazanıyor. Sapma çözücüde
değil, ona verilen örneklerde.

**Ayırt edici imza koşullanma duyarlılığıdır.** Gözlemde koşullanma 17 kat
değişirken hata kıpırdamamıştı. Her aday iki poz kümesinde (geniş/dar)
koşuldu; doğru adayın hem büyüklüğü hem de **oran ≈ 1.00**'ı tutturması gerekir:

| Aday konvansiyon hatası | geniş | dar | oran | Sonuç |
| --- | ---: | ---: | ---: | --- |
| A frame'i sabit ötelemeyle kayık | 9.000 | 9.000 | **1.00** | ✅ imzayı üretir |
| Kamera tarafı sabit ötelemeyle kayık | 9.000 | 9.000 | **1.00** | ✅ imzayı üretir |
| B ters (board→camera) | 168.3 | 331.1 | 1.97 | ❌ elendi |
| A ters (wrist→base) | 152.3 | 139.8 | 0.92 | ❌ büyüklük tutmuyor |
| Board ölçeği %2 büyük | 2.20 | 6.22 | 2.82 | ❌ koşullanmaya duyarlı |
| Board ölçeği %0.5 büyük | 0.55 | 1.56 | 2.82 | ❌ aynı sebeple |
| B saf dönme hatasında (optik frame) | 0.000 | 0.000 | — | ❌ etkisi yok |

**Sonuç:** sapma, X'in **bir ucundaki sabit öteleme** sınıfındandır ve hata tam
olarak o ötelemeye eşittir. Gürültü ve ölçek sınıfları elendi, çünkü ikisi de
koşullanmayla ölçekleniyor.

**Ölçüm hangi uç olduğunu SÖYLEYEMEZ:** bilek tarafındaki 9 mm ile kamera
tarafındaki 9 mm birebir aynı imzayı veriyor. Bunu yalnız frame zincirini
okumak ayırır.

**Yan bulgu:** B'deki saf dönme hatası X'in ötelemesini hiç bozmuyor. Yani
optik frame konvansiyonu karışıklığı bu sapmayı açıklayamaz — ve daha önemlisi,
bu tür bir hata öteleme metriğinde **görünmez**.

### Modelde bakılıp elenen somut ofsetler

- `tool0` ↔ `link_5`: `[0.09475, -0.07099, -0.00352]` → **118 mm**, 9 değil.
- Sim kamera sensörü: `camera_link`'e **pozsuz** bağlı (`robot_arm.urdf.xacro`),
  yani optik merkez ofseti yok.
- Yakalama `base_link → link_5` kullanıyor, `X_GT` de mount ∘ optik dönüş
  olarak `link_5`'ten türüyor — bu ikisi tutarlı.

**Kalan iş:** zincirde ~9 mm'lik sabit bir öteleme aramak. Metrik onu bulmaz;
yalnız prova kodundaki frame kullanımını satır satır okumak bulur. Bu, gerçek
kolda hand-eye koşmadan önce kapanmalı: aynı hata orada **yer gerçeği
olmadığı için sessizce geçer.**


### ⚠️ DÜZELTME (aynı gün, 2026-08-15): "sabit ofset" sonucu ÇÜRÜDÜ

Yukarıdaki tablo doğru ama ondan çıkardığım sonuç yanlıştı. Eksik olan aday
**B'nin AÇISAL gürültüsüydü** ve tam da bu belgenin baskın kanal ilan ettiği şey
oydu. Sadece öteleme/ölçek sınıflarını denedim, açısal olanı denemedim.

**Kesin test — provanın GERÇEK poz kümesiyle** (`data/hand_eye/sim_wrist_poses.json`,
5 poz), yer gerçeği X ile kusursuz B üretilip üzerine ölçülmüş açısal taban
enjekte edildi (200 deneme, PARK):

| Enjekte edilen açısal gürültü | X hatası medyan | p10 | p90 |
| ---: | ---: | ---: | ---: |
| 0.10° | 1.67 mm | 0.82 | 3.15 |
| 0.25° | 4.18 mm | 2.01 | 7.87 |
| **0.46° (ölçülen taban)** | **7.74 mm** | 3.70 | 14.44 |
| 0.70° | 11.85 mm | 5.67 | 21.92 |
| 1.00° | 16.99 mm | 8.21 | 31.21 |

Gözlenen 8.92 / 9.34 / 9.07 mm bu dağılımın **tam içinde**. Gürültüsüz aynı
kümede hata 0.0000 mm, yani poz kümesi tek başına da suçlu değil — çarpan
gürültü × koşullanma.

**Sonuç: gizli bir 9 mm'lik frame ofseti postüle etmeye gerek yok.** Sapma,
zaten ölçülmüş açısal tabanın 5 pozluk bir kümedeki normal sonucudur.

**Hatalı akıl yürütmem neydi:** "hata koşullanmayla ölçeklenmiyor, demek ki
gürültü değil" dedim. İki kusuru vardı. (1) Koşullanma göstergesi olarak
kayıttaki **yöntem yayılımını** aldım; ölçtüm ki yayılım TSAI'nin dar kümelerde
tamamen çökmesinden domine oluyor ve hatayı sadakatle takip etmiyor. (2) Üç
koşu bağımsız gürültü çekilişleri değildi — aynı sim, aynı/benzer pozlar,
deterministik render; birbirine yakın çıkmaları beklenirdi.

**Ayakta kalan:** çözücünün aklanması (kusursuz veride 0.0000 mm) ve elenen
adaylar (B ters, A ters, board ölçeği, saf dönme). Düşen: "sabit öteleme"
teşhisi.

**Sıradaki işe etkisi — yön değişti.** Aranacak bir montaj hatası yok; yapılacak
şey bu belgenin zaten söylediğidir: **açısal tabanı küçült ve poz kümesini
çoğalt/çeşitlendir.** 5 poz bu taban için yeterli değil. Ve tabanın kendisi
gerçek kolda hâlâ ölçülmedi (adım 3a).


### Mevcut kümenin bugünkü kısıtlara göre denetimi (2026-08-15)

`data/hand_eye/gated_pose_set.json` (20 poz), bugün ölçülen iki kısıta karşı
denetlendi. **Küme yeniden üretilmedi; ölçüldü.**

| Kısıt | Ölçüm | Sonuç |
| --- | --- | --- |
| Fronto-paralel bakıştan kaçın (0 derecede 1.77 derece poz hatası) | eğim min **45.3**, medyan 58.8, maks 82.7 derece; 10 derecenin altında **0/20** poz | ✅ geçiyor, rahatça |
| Pozlar arası düzlem-içi dönme farkını küçük tut (flip marjı yapısal olarak sıfır) | maks ikili fark **91.1 derece**; 90 dereceyi aşan **2 çift** | ⚠️ iki çift sınırda |

Yani bugünkü fronto-paralel bulgusu bu kümeyi vurmuyor — o bulgu poz
ÜRETİMİ ve 399'luk havuz için geçerli (havuzun ~%14'ü belirsizliğe düşüyordu).
Düzlem-içi dönme tarafında ise iki çift, sıralamanın dönebileceği bölgede.

**Not:** `hand_eye_pose_search.py` bu iki kısıtın hiçbirini bilmiyor (kaynakta
eğim/düzlem-içi dönme geçmiyor). Küme bugün geçiyor olsa da, üreteci yeniden
koşan biri kısıtları ihlal eden bir küme alabilir. Üretecin sertleştirilmesi
açık iş; küme geçtiği için acil değil.

### ⚠️ DÜZELTME (2026-08-18): yukarıdaki denetim YANLIŞ FRAME'DE ölçülmüş

Üreteci sertleştirmeye başlarken ilk iş, 15 Ağustos denetiminin sayılarını
yeniden üretmekti — tanımlar aynı olsun diye. **Üretilemedi.** Denetim ad hoc
koşulmuştu, betiği commit edilmemişti; makul her tanım denendi ve hiçbiri
tabloyu vermedi.

Sebep bulundu: denetim tahtayı `hand_eye_convention_check.py`'deki **sentetik
`BOARD` sabitinden** kurmuş. O sabit konvansiyon testinin kurgusal tahtasıdır,
yakalamanın gerçek tahtası değil — ikisi baz frame'de ~11 cm ayrı duruyor.
Doğru kaynak zaten elimizde: yakalanmış örneklerin **ölçülmüş `cam_to_board`**
alanı. Tahtanın kameradaki pozu için ara model kurmaya gerek yok, o poz
doğrudan ölçülmüş.

Ölçülmüş `cam_to_board` ile aynı iki kısıt:

| Kısıt | 2026-08-15 denetimi | ÖLÇÜLEN (`gated_samples.json`) |
| --- | --- | --- |
| eğim (0 = tam cepheden) | min 45.3, medyan 58.8, maks 82.7 | **min 0.6, medyan 5.4, maks 18.2** |
| 10 derecenin altındaki poz | 0/20 | **16/20** |
| düzlem-içi dönme, maks ikili fark | 91.1 derece, 2 çift > 90 | **111.7 derece, 16 çift > 90** |

**Verdict tersine döndü.** Küme fronto-paralel kısıtını "rahatça geçmiyor";
tam tersine ağırlıklı olarak dejenere bölgede duruyor. 399'luk havuzda da
**359/399** poz 10 derecenin altında — yani havuzun %90'ı. Bu, daha önce aynı
havuzda ölçülen "%14'ü çift-çözüm belirsizliğine düşüyor" bulgusuyla tutarlı ve
onu açıklıyor: belirsizlik cepheye yakın bakışta yoğunlaşıyor, havuz da zaten
cepheye yakın bakışlardan oluşuyor.

İki bağımsız yol aynı sonucu veriyor: (1) ölçülmüş `cam_to_board`, (2) `pool`
komutunun referans yakalamadan kestirdiği tahta pozu — yeni kapı, kadrajda
kalan 1555 adayın **1405'ini** cepheye çok yakın diye eliyor.

Bunun ölçülen hataya etkisi spekülasyon değil, aynı belgede duruyor: 0 derece
eğimde poz hatası 1.77 derece ve 0.5 derece ≈ 2.9 mm X hatası. Kümenin gerçek
veriyle ölçülen hatası 2.81 mm idi. Yani "açısal taban" olarak kaydedilen şeyin
bir kısmı, kümenin kendi bakış geometrisinden geliyor olabilir — bu ayrıştırma
henüz yapılmadı.

**Kapanan iş:** `hand_eye_pose_search.py` artık iki kısıtı da biliyor ve
UYGULUYOR (yalnız raporlamıyor):

- `pool` — kadraj kestirimine ek olarak eğim kapısı; elenenler kadraj/cephe
  diye ayrı sayılıyor. Ev pozu (0,0,0,0,0) da artık muaf değil; önceki sürüm
  onu her havuzun başına kayıtsız şartsız koyuyordu.
- `select` — eğim kapısını yeniden uygular (kapı öncesi üretilmiş havuzlar
  mevcut), düzlem-içi dönme farkını arama boyunca kısıt olarak tutar, ve
  başlangıç kümesini de kısıtlı kurar. Kısıt sağlanamıyorsa en fazla kaç poz
  bir arada seçilebildiğini söyleyerek durur.
- `--min-tilt-deg` / `--max-inplane-deg` ile gevşetilebilir; ikisi de 0'da
  hangi riski geri getirdiğini yazdırır.
- `select --out` eklendi: varsayılan hedef KANONİK `gated_pose_set.json`'dı ve
  her deneme koşusu onu eziyordu.

**Açık kalan:** mevcut `gated_pose_set.json` bu kapılardan geçmiyor. Yeniden
üretilmesi tahtanın yerleşimini değiştirmeyi gerektirebilir — bu geometride
kolun ulaşabildiği ve tahtayı kadrajda tutan pozların %90'ı cepheden bakıyor.
Asıl çözüm ChArUco'dur ve bu belge onu zaten öneriyor.

### Kısıtların bedeli ölçüldü — doğruluğu KURTARMIYOR, sessiz bir arıza modunu kaldırıyor

Sertleştirilmiş üreteç aynı 399'luk havuza uygulandı. Eğim kapısı havuzun
**40/399**'unu bırakıyor; bu 40 içinden düzlem-içi dönme kısıtını da sağlayan
20'lik bir küme bulunabiliyor (seçilen kümede eğim min 10.2 derece, en büyük
ikili dönme farkı 87.7 derece). Yani kısıtlar sağlanabilir.

Ama beklenen hata **kötüleşiyor**, çünkü havuz 10 kat daralınca koşullanma
bozuluyor. Aynı gürültü modeliyle (0.47 mm öteleme, 0.46 derece dönme):

| Küme | beklenen X hatası ort / p90 |
| --- | --- |
| mevcut `gated_pose_set.json` (kısıtsız) | 2.69 / **4.43** mm |
| kısıtlı arama, aynı havuz | 3.43 / **6.00** mm |

Eğim taramasının ölçtüğü dejenerasyon modele *poz başına* sigma olarak
işlendiğinde (0 derecede 1.77, 5 derecede 0.04, tabanla birlikte alt sınır
0.46) mevcut küme 2.89 / 5.02 mm'ye çıkıyor — yani cepheden bakmanın bedeli
modelde görünüyor ama farkı kapatmıyor. **İki küme de 1.0 mm hedefinin çok
uzağında.**

Sonuç: bu kısıtlar doğruluk kazandırmıyor. Yaptıkları şey, reprojeksiyonun ve
flip marjının **yapısal olarak göremediği** bir hata sınıfını poz kümesinden
çıkarmak. Doğruluk için gereken şey başka yerde: tahtanın yerleşimi (bu
geometride kadrajda kalan pozların %90'ı cepheden bakıyor) ve ChArUco.

## Eğim kapısı GERÇEK kamerada ölçüldü (2026-08-18)

`MIN_TILT_DEG = 10` bugüne dek bir belgedeki **sentetik** taramadan geliyordu.
Artık kameradan geliyor. Sabitleme imkânı olmadığı için statik ölçüm yapılamadı;
gerekmedi de — çift-çözüm belirsizliği **kare başına** bir özelliktir, tahta
elde gezdirilerek taranabilir. Araç: `scripts/tilt_ambiguity_sweep.py`.

İki koşu, toplam **1745 kare**, eğim 3.6–47.9 derece. Her kare için IPPE'nin iki
çözümü alınıp reprojeksiyon oranları karşılaştırıldı (oran < 1.5 = reprojeksiyon
ikisi arasında seçemez).

| eğim | kare | oran medyan | oran p10 | belirsiz | çözüm farkı |
| --- | ---: | ---: | ---: | ---: | ---: |
| 6–8 | 14 | 1.20–2.51 | 1.05 | **%86** | 12–15° |
| 8–10 | 8 | 2.46–2.82 | 2.47 | %0 | 18–19° |
| 10–11 | 69 | 4.84 | **3.66** | %0 | 21° |
| 11–15 | 851 | 5.6→8.5 | 4.8→7.8 | %0 | 23–29° |
| 15–20 | 533 | 9.7→19.4 | 8.9→13.4 | %0 | 31–39° |
| 20–48 | 270 | 22.6→31.9 | — | %0 | 42–94° |

**Kesin olan:** 10 derecenin üstünde, **1719 karede sıfır belirsiz kare.** Oran
eğimle birlikte tekdüze büyüyor ve 10–11 bandında en kötü %10'luk dilimde bile
3.66 — rahat pay. Gözlenen bütün belirsiz kareler (12 tane) **6.0–7.8 derece**
arasında. Yani kapının 10'da durması ölçümle destekleniyor.

**Bedeli de ölçüldü:** yanlış çözüm marjinal değil. 10–11 derecede iki çözüm
arasındaki dönme farkı **21 derece**. Belirsiz bir kare kabul edilirse hata
küçük olmaz, poz tamamen başka yere gider.

**Kesin OLMAYAN — kayda geçsin:**
- 12 belirsiz kare ardışık bir el hareketinden geliyor (eğim 6.0→7.8 monoton),
  yani 12 bağımsız örnek değil, **tek bir geçiş**.
- Aynı kareler en bulanık olanlar: belirsizlerin reprojeksiyon medyanı 0.857 px,
  diğerlerinin 0.352. Ama bulanıklık tek başına açıklamıyor — reproj > 0.5 px
  olan 81 karenin yalnız %15'i belirsiz, reproj ≤ 0.5 olan 1664 karenin **%0**'ı.
  Mesafe fark etmiyor (0.53 / 0.55 m). Yani düşük eğim ile bulanıklık bu veride
  **ayrıştırılamadı**.
- 8–10 derece bandında 8 kare var; sınırın tam yeri belirlenemez.
- Sentetik taramanın en kötü dediği bölge (0–2 derece) **hiç örneklenmedi** —
  3.6 derecenin altına inilmedi.

Kapı 10'da kaldığı sürece bu boşluklar kararı değiştirmiyor; kapıyı düşürmek
isteyen biri önce 0–10 derece bandını doldurmak zorunda.

## Öteleme tabanı DERİNLİK tabanıdır, ve mesafeyle değil GÖRÜNTÜDEKİ BOYUTLA belirlenir (2026-08-18)

Açısal taban öne çıkınca öteleme sayısı artifact'te öylece kalmıştı. Modele
sokulunca kayıtlı bir sonuç düştü, sonra da düzeltme önerim ölçümle çürüdü.

**1. Ölçülen çift, varsayılan çiftten farklı.** Model 0.47 mm / 0.46 derece
taşıyordu; ölçülen 1.313 mm / 0.220 derece — öteleme 3 kat kötü, açı 2 kat iyi.
Mevcut 20 pozluk kümede her kanal tek başına: öteleme 2.37 mm p90, açı 2.19 mm
p90. **Yani "baskın kanal DÖNME" sonucu bu donanımda geçerli değil** (o sonuç
0.41 mm / 0.5 derece varsayımından çıkmıştı). Toplam beklenen hata 4.43 → 3.30 mm.

**2. Öteleme saçılımının tamamı derinlik.** Araca eklenen ayrım (bakış hattı
boyunca / dik) ölçtü:

| koşu | öteleme p90 | derinlik p90 | yanal p90 | açısal p90 |
| --- | ---: | ---: | ---: | ---: |
| ilk yerleşim | 1.313 | — | — | 0.220 |
| tahta "50 cm"e taşındı | 1.266 | **1.252** | **0.289** | 0.249 |

Yanal bileşen derinliğin beşte biri. Tek kamerayla düzlemsel hedefte beklenen
davranış bu, ama artık ölçülü.

**3. Yaklaştırma önerim yanlıştı — düzeltiliyor.** "Derinlikse tahtayı
yaklaştırmak düşürür" dedim ve mesafeyi yarıya indiren bir yerleşim istedim.
Taban **kıpırdamadı** (1.313 → 1.266). Sebep ölçüldü: tahtanın GÖRÜNTÜDEKİ
boyutu değişmedi.

| | ilk yerleşim | "50 cm" yerleşimi |
| --- | --- | --- |
| köşe aralığı | 13.1 / 11.5 px | 14.0 / 11.8 px |
| tahta kutusu | 76 × 89 px | 75 × 96 px |

Aynı piksel ayak izi → aynı derinlik tabanı. **Derinlik tabanını belirleyen şey
fiziksel mesafe değil, tahtanın kaç piksel kapladığıdır.** Yaklaştırmak ancak
ayak izini büyütüyorsa işe yarar; burada büyütmedi, yani 50 cm'ye konan tahta
fiziksel olarak daha küçük bir baskı olmalı.

Tahta şu an 640×480 karenin **75×96 pikselini** kaplıyor — alanın ~%2.3'ü.
Yapılacak iş yaklaştırmak değil, **kadrajı doldurmak**: daha büyük baskı ve/veya
ayak izi gerçekten büyüyecek kadar yakın. Sonra bu ölçüm tekrarlanır.

**⚠ AÇIK: kare boyutu cetvelle DOĞRULANMADI.** `board_pnp` 27.5 mm varsayıyor ve
bu bir baskı-ölçeği tahmininden geliyor. Kullanıcı tahtayı 50 cm'ye koydu, PnP
1.01 m ölçtü — oran 2.02. Ölçek hatası bütün mm çıktılarını aynı katsayıyla
kaydırır ve **reprojeksiyonda görünmez** (0.24 px, kusursuz duruyor);
`make_checkerboard.py` docstring'i tam bunu uyarıyor. Bu belgedeki mm sayıları
o doğrulama gelene kadar ölçek belirsizliği taşır. **Açısal sonuçlar etkilenmez.**

### ⚠️ DÜZELTME (aynı gün): ölçek doğrulandı, "piksel tabanı" iddiası ise ÖLÇÜLMEDİ

Kullanıcı iki şeyi bildirdi: kare cetvelle **27.5 mm** (yani `board_pnp` doğru
varsayıyor), ve tahta **tam 50 cm'e ayarlanmadı**.

**Kapanan:** yukarıdaki ölçek uyarısı geçersiz. 2.02 oranı bir ölçek hatası
değildi; PnP'nin 1.01 m'si doğru, tahta gerçekten orada. Bu belgedeki mm
sayıları ölçek belirsizliği TAŞIMIYOR.

**Çöken:** "yaklaştırmak işe yaramadı, demek ki taban görüntüdeki boyutu takip
ediyor" çıkarımı. O çıkarımın kanıtı "mesafe yarıya indi ama ayak izi
değişmedi"ydi. Mesafe inmedi — iki koşu da ~1.0 m'de, 75×96 px'te yapıldı.
Yani elimizdeki şey deney değil, **aynı geometride iki tekrar**:

| koşu | mesafe | ayak izi | öteleme p90 | derinlik p90 | açısal p90 |
| --- | --- | --- | ---: | ---: | ---: |
| a | ~1.04 m | 76 × 89 px | 1.313 | — | 0.220 |
| b | ~1.01 m | 75 × 96 px | 1.266 | 1.252 | 0.249 |

Bu haliyle değerli: taban **tekrarlanabilir**, iki bağımsız 60 karelik koşu
aynı sayıyı veriyor. Ama "kadrajı doldurmak tabanı düşürür" hâlâ bir
HİPOTEZ; ölçülmedi.

**Ayakta kalan ve ölçülmüş olan:** öteleme saçılımı derinlik ağırlıklı
(1.252 / 0.289 p90) ve ölçülen çift (1.313 mm / 0.220 derece) modelin taşıdığı
çiftten (0.47 / 0.46) farklı, dolayısıyla "baskın kanal dönme" sonucu geçersiz.

**Hipotezi sınayan koşu:** tahtayı ayak izi GERÇEKTEN büyüyecek kadar yaklaştır
ve hareketi `board_tilt_live.py`'nin mesafe okumasından DOĞRULA — bu sefer
tahminle değil. ~0.5 m'de ayak izi ~150×190 px olmalı, yani alan 4 kat. Derinlik
tabanı buna rağmen düşmezse hipotez yanlıştır ve taban başka bir yerden geliyor.

## Hipotez sınandı: kadrajı doldurmak derinlik tabanını 3.6 kat düşürüyor (2026-08-18)

Bu sefer geometri değişikliği ÖLÇÜLDÜ, varsayılmadı: mesafe 1.02 → **0.50 m**,
köşe aralığı 14.0/11.8 → **28.4/27.9 px**, ayak izi 75×96 → **149×227 px**
(doğrusal 2 kat, alan 3.7 kat).

| | 1.02 m (75×96 px) | 0.50 m (149×227 px) |
| --- | ---: | ---: |
| öteleme p90 | 1.266 mm | **0.434 mm** |
| derinlik p90 | 1.252 mm | **0.345 mm** |
| yanal p90 | 0.289 mm | 0.291 mm |
| açısal p90 | 0.249° | **0.180°** |
| reprojeksiyon medyan | 0.241 px | 0.500 px |

**Derinlik 3.6 kat düştü.** Hata görünür boyutun karesiyle gitseydi 4 kat
beklenirdi; 3.6 geldi. Hipotez doğrulandı ve sayısallaştı.

**Yanal değişmedi, ve bu tutarlılık kanıtı.** Yanal hata ≈ piksel hatası × Z/f.
Z yarıya indi, piksel hatası (reproj 0.241 → 0.500) iki katına çıktı, ikisi
birbirini götürdü. Reprojeksiyondaki kötüleşme muhtemelen IMX219'un sabit odağı
— 0.5 m odak menzilinin kenarında. Yani kural "yaklaş" değil, **"kadrajı doldur
AMA odakta kal"**; odak kaybı yanal kazancı tam olarak siliyor, derinlik kazancı
ise hayatta kalıyor.

**Kural olarak:** hand-eye yakalamasında tahta kadrajın küçük bir yüzdesini
kaplıyorsa derinlik tabanı boşuna büyür. Ayak izi bir kabul ön koşuludur ve
`measure_perception_floor.py` çıktısındaki derinlik/yanal ayrımından okunur.

### ⚠ Hedefe göre konum: cevap hedefin İKİ YANINA düşüyor

Mevcut 20 pozluk kümede beklenen X hatası:

| taban | ort | p90 |
| --- | ---: | ---: |
| modelin eski varsayımı (0.47 mm, 0.46°) | 2.69 | 4.43 mm |
| 1.0 m tabanı (p90 çifti) | 1.93 | 3.42 mm |
| 0.5 m tabanı (**p90** çifti) | 1.12 | **1.93 mm** |
| 0.5 m tabanı (**medyan** çifti) | 0.54 | **0.92 mm** |

Sabahki "1.0 mm bu donanımla ulaşılamaz" hükmü artık geçerli değil. Ama hedefin
geçildiği de söylenemez, çünkü fark tamamen **hangi istatistiğin modele σ olarak
verildiğine** bağlı: medyan çiftiyle 0.92 (geçer), p90 çiftiyle 1.93 (kalır).

Bu eşleme bugüne dek önemsizdi çünkü sonuç her iki okumada da hedefin çok
uzağındaydı. Artık önemli. Ölçülen şey merkeze olan UZAKLIKLARIN dağılımı;
model ise eksen başına σ tüketiyor ve izotropik gürültüde bu ikisi sabit bir
katsayıyla ilişkili (Maxwell dağılımı). Eşlemenin türetilip doğrulanması
**açık iş** ve hedefin geçilip geçilmediği ona bakıyor.

## Saçılım → σ eşlemesi türetildi; hüküm p90 1.40 mm (2026-08-18)

Eşleme kapalı formda yapılamadı, ölçüldü. Araç artık ham kareleri de saklıyor.
120 kare, 0.499 m, tahta sabit, kol hareketsiz.

**Varsayımsız σ (örnek std):**

| | değer |
| --- | --- |
| öteleme, izotropik eşdeğer (rms/√3) | **0.1728 mm** |
| — derinlik ekseni | 0.2176 mm |
| — yanal eksenler | 0.1460 mm |
| dönme (ölçülen \|açı\| rms) | **0.1434 derece** |

**Dağılım sınaması:** derinlik basıklığı 3.56 (Gauss 3), çarpıklık −0.05,
lag-1 ardışık korelasyon derinlikte −0.06 / açıda 0.17, ilk yarı–ikinci yarı
kayması ±0.0015 mm. Yani kareler pratikte bağımsız, dağılım Gauss'a yakın,
sistematik sürüklenme yok.

**Kapalı form neden yetmedi:** bu veride ampirik olarak σ = medyan/1.115 =
p90/2.663, izotropik Gauss teorisi ise 1.538 / 2.500 diyor. Gürültü anizotropik
(derinlik yanalın 1.5 katı) ve anizotropi medyan uzaklığı rms'e göre küçültüyor.
Ham örnek yolu gerekliydi.

**Beklenen X hatası (mevcut 20 pozluk küme, 600 çekiliş):**

| model | ort | medyan | p90 |
| --- | ---: | ---: | ---: |
| izotropik σ 0.173 mm + 0.143° | 0.82 | 0.71 | **1.40 mm** |
| anizotropik (0.218 / 0.146) + 0.143° | 0.82 | 0.70 | 1.48 mm |

Anizotropiyi modellemek sonucu değiştirmiyor (rms korunuyor); izotropik eşdeğer
yeterli.

### HÜKÜM

Belgenin kendi kuralı kapıyı **p90** üzerine kuruyor ("kötü ama makul bir
çekilişte bile hedef tutmalı"). Buna göre **1.40 mm > 1.0 mm: hedef GEÇİLMEDİ.**
Ama sabahki 4.43 mm'lik tablodan çok uzakta — eksik olan iyileşme 4.4 kat değil,
**1.4 kat**. Ortalama zaten 0.82 mm ile hedefin altında; kapıyı p90 tuttuğu için
geçmiyor.

### ⚠️ DÜZELTME: "baskın kanal dönme değil" sonucum YANLIŞTI

Bugün erken saatte, ölçülen çiftin modelin varsayımından farklı olmasına
dayanarak belgenin "baskın kanal DÖNME" sonucunu çürüttüğümü yazmıştım. O
çürütme geçersiz. Doğru σ ile kanal ayrımı:

| kanal | p90 |
| --- | ---: |
| ikisi birden | 1.40 mm |
| yalnız öteleme | **0.31 mm** |
| yalnız dönme | **1.44 mm** |

Dönme ezici biçimde baskın; öteleme neredeyse hiç katkı vermiyor. **Belgenin
2026-08-14'teki orijinal sonucu doğruymuş.**

Hatanın kaynağı öğretici: saçılım istatistiklerini iki kanalda da σ yerine
kullanmıştım. Ama öteleme saçılımı 3B bir UZAKLIK (σ'nın ~1.5 katı), dönme
saçılımı ise 1B bir BÜYÜKLÜK (σ'nın ~0.8 katı). İkisini aynıymış gibi
karşılaştırmak ötelemeyi sistematik olarak şişiriyor. Eşleme türetilmeden
yapılan kanal karşılaştırması bu yüzden anlamsızdı.

### Bundan sonra nereye yüklenilecek

p90 ≤ 1.0 mm için gereken σ ölçeği ölçüldü:

| σ çarpanı | öteleme | dönme | p90 |
| --- | --- | --- | ---: |
| ×1.0 | 0.173 mm | 0.143° | 1.40 mm |
| ×0.8 | 0.138 | 0.115° | 1.12 mm |
| **×0.7** | 0.121 | **0.100°** | **0.98 mm — geçer** |

Yani **%30'luk bir iyileşme yetiyor ve tamamı DÖNME kanalından gelmeli.**
Somut aday: bu koşuda reprojeksiyon medyanı 0.496 px (1.0 m'deki 0.24'ün iki
katı), muhtemelen IMX219'un sabit odağı 0.5 m'de zorlanıyor. Köşe
lokalizasyonunu düzeltmek doğrudan dönme tabanına yazılır. Odak/aydınlatma
düzeltilip bu ölçüm tekrarlanmalı; ChArUco da aynı kanala etki eder.

## Sistematik artık TAHTADA değil, INTRINSICS'te (2026-08-18)

0.65 m'de reprojeksiyon 0.50–0.55 px çıkınca sebep arandı. İlk ölçüm artığın
gürültü olmadığını gösterdi: tahta koordinatında bir kuadratik artığın
**%90/%80**'ini açıklıyor, kareler arası dalgalanma ise yalnız 0.072–0.085 px
iken ortalama artık 0.19–0.31 px. Yani sabit, düzgün, uzamsal bir desen.

İki aday vardı — tahtanın düz olmaması, ya da lens/distorsiyon modeli — ve
**tek görünüm ayırt edemez**, çünkü tek görünümde tahta ve görüntü koordinatları
arasındaki dönüşüm neredeyse afindir (ikisinde de %91/%81 çıktı).

Ayrım iki görünümle yapıldı, **aynı mesafede** (büyütme oranı 1.01):

| görünüm | mesafe | eğim | ortalama artık |
| --- | --- | --- | --- |
| a | 0.65 m | 6.6° | 0.307 px |
| c | 0.65 m | 35.1° | 0.189 px |

**Tahta koordinatındaki desen korelasyonu: +0.11.** Desen tahtayla taşınmıyor,
görüntüde sabit kalıyor → kusur **intrinsics/distorsiyon modelinde**.
Araç: `scripts/board_residual_probe.py`.

**Bağımsız doğrulayıcılar:**
- Distorsiyon katsayıları salınıyor: `[0.258, −0.753, −0.002, 0.003, 0.731]`.
  k1 pozitif, k2 güçlü negatif, k3 güçlü pozitif — yetersiz/kötü dağılmış
  görüntüyle aşırı uydurulmuş modelin imzası.
- `board_pnp` kaynağı intrinsics'in **eski 25 mm tahtayla** alındığını zaten
  yazıyor; `capture_calib_images.py`'nin örneği de 6x8 / 25 mm. Mevcut tahta
  6x9 / 27.5 mm.

### ⚠ Bunun σ hesabına etkisi: 1.40 mm p90 İYİMSER olabilir

Türetilen σ_dönme = 0.143 derece, **tek pozda kare-arası saçılımdır**. Distorsiyon
hatası ise kare-arası gürültü değil, **poza bağlı SİSTEMATİK sapma** üretir: her
poz görüntünün başka bir yerine düşer, başka bir sapma alır. Hand-eye çözücüsü
bunu pozlar arası tutarsızlık olarak görür — yani gürültü gibi davranır — ama
tek pozda yapılan taban ölçümü onu **hiç göremez**.

Büyüklüğü için bir gösterge: yalnız k3'ü sıfırlamak pozu görünüme göre
0.003–0.063 derece değiştiriyor, görünümler arası yayılım **0.060 derece**.
Bu, ölçülen 0.143 derecelik saçılımın azımsanmayacak bir kesri ve modele
hiç girmiyor. Tek bir katsayının etkisi olduğu için de tam bir muhasebe değil.

**Sonuç:** hedef hükmü (p90 1.40 mm) bu kanalı içermiyor, dolayısıyla gerçek
değer daha kötü olabilir. Ve gereken %30'luk iyileşmenin dönme kanalından
gelmesi gerektiği hatırlanınca, **intrinsics'i yeniden kalibre etmek** hem bu
gizli kanalı kapatan hem de dönme tabanını düşüren tek hamle olarak öne çıkıyor.
Kol hareketi gerektirmez; araçlar mevcut (`capture_calib_images.py` +
`solve_camera_intrinsics.py`, ikincisi ayrılmış doğrulama kümesi zorunlu tutuyor).

## Dedektör değişimi hedefi geçirdi: p90 1.40 → 0.63 mm (2026-08-18)

Intrinsics'i yeniden kalibre etme hipotezi **çürüdü**, ama arayış doğru yere
götürdü. `solve_camera_intrinsics.py` 2026-07-22'de sektör tabanlı
`findChessboardCornersSB`'ye geçmiş ve gerekçesini yazmış: *"legacy dedektör +
cornerSubPix, IMX219'un ISP ile yeniden ölçeklenmiş 640x480 akışında
çok-piksellik sistematik sapma gösterdi"*. **`board_pnp` bu geçişi hiç almamıştı**
ve bütün ölçüm hattı ondan besleniyordu.

25 kalibrasyon karesinde, aynı intrinsics ile:

| dedektör | reproj ort | medyan | maks | Nano maliyeti |
| --- | ---: | ---: | ---: | ---: |
| legacy | 0.450 px | 0.291 | **2.561** | **981 ms/kare** |
| SB | **0.279 px** | 0.234 | **0.590** | **209 ms/kare** |

Taviz yok: daha doğru, kuyrukta 4.3 kat daha iyi, ve 4.7 kat daha hızlı.
`board_pnp` SB'ye geçti; legacy yol yalnız OpenCV < 4.0 yedeği olarak kaldı.

**Aynı geometride (0.50 m, 120 kare) σ karşılaştırması:**

| | legacy | SB |
| --- | ---: | ---: |
| σ dönme | 0.1434° | **0.0546°** |
| σ öteleme | 0.1728 mm | 0.1868 mm (rüzgâr kirli) |
| — derinlik | 0.2176 | 0.1493 |
| reprojeksiyon | 0.496 px | 0.323 px |

**Beklenen X hatası: p90 1.40 → 0.63 mm. Hedef (1.0 mm) GEÇİLDİ**, üstelik
öteleme σ'sı şişmiş haliyle kullanıldığı için muhafazakâr.

### Rüzgâr ölçüldü, ayrıştırıldı

Kullanıcı koşu sırasında hafif titreşim olabileceğini bildirdi. Ham kareler
saklandığı için bu sınandı — algı gürültüsü beyazdır, mekanik hareket zamanda
ilişkilidir:

| büyüklük | lag-1 | lag-2 | lag-5 |
| --- | ---: | ---: | ---: |
| yanal | **+0.48** | +0.46 | +0.35 |
| derinlik | **+0.31** | +0.35 | +0.21 |
| açı | +0.12 | +0.09 | +0.04 |

(gürültü seviyesi ±0.09.) Öteleme kanalı kirli, **dönme kanalı temiz** —
ve hüküm dönme kanalına dayandığı için sonuç ayakta. Açı ile öteleme
korelasyonu −0.14/−0.15, yani rijit sarkaç değil yavaş kayma. Yanal σ'nın
legacy'ye göre kötüleşmesinin (0.146 → 0.203) sebebi budur, dedektör değil.

### ⚠ Hâlâ modelde OLMAYAN kanal — ve artık baskın olabilir

Distorsiyonun ürettiği **poza bağlı sistematik sapma** bu modele hiç girmiyor;
tek pozda ölçülen σ onu göremez. Daha önce ölçülen göstergesi: yalnız k3'ü
sıfırlamak pozu görünüme göre 0.003–0.063 derece değiştiriyordu, yayılım
**0.060 derece**.

Bu, σ_dönme 0.143 iken ihmal edilebilirdi. **Şimdi σ_dönme 0.055 ve o yayılım
onunla aynı mertebede.** Yani sıralama tersine döndü: intrinsics işi, dedektör
düzeldikten sonra kalan en büyük şüpheli hâline geldi.

Kalibrasyon tarafında bugün ölçülen: aday intrinsics ayrılmış sette eskisini
**yalnız %5** yeniyor (SB köşeleriyle 0.2098 vs 0.2209 px; legacy köşelerle
yapılan ilk karşılaştırma 0.2716 vs 0.2724 ile "fark yok" diyordu ve o ölçüm
legacy dedektörün kendi sapmasıyla maskelenmişti). Aday YAML çözücünün kendi
kapılarını geçmedi (18 fit karesi < 25, 7 doğrulama < 10, üstelik tek koşudan
bölünmüş) ve **etkinleştirilmedi**.

## ⛔ HÜKÜM GERİ ALINDI: bağlayıcı kısıt algı değil KALİBRASYON (2026-08-18)

Bağımsız doğrulama setiyle (25 fit + 15 ayrı koşu) yeni intrinsics çözüldü ve
çözücünün kapılarını geçti. Ama iki kalibrasyon karşılaştırılınca ortaya çıkan
şey, hükmü tersine çevirdi.

**Eski ve yeni intrinsics reprojeksiyonda ayırt edilemiyor ama POZDA anlaşamıyor:**

| | değer |
| --- | ---: |
| bağımsız doğrulama, eski | 0.2932 px |
| bağımsız doğrulama, yeni v2 | 0.2669 px (yalnız %9 iyi) |
| eski↔yeni **poz** farkı | ort 0.677°, maks **1.858°** |

**Ve bu bir kalibrasyon çifti kazası değil.** Aynı 25 karelik veriden 20'şerlik
alt kümelerle 12 bootstrap kalibrasyonu üretildi (`scripts/calibration_stability.py`):

| | değer |
| --- | ---: |
| hepsinin reprojeksiyon RMS'i | 0.206 – 0.251 px (**ayırt edilemez**) |
| fx yayılımı | 484.1 – 517.4 (**%6.6**) |
| doğrulama karesi başına poz yayılımı | medyan **0.540°** / p90 0.605 |
| ölçülen algı σ_dönme | **0.0546°** |
| **oran** | **9.9×** |

Yani kabul metriği (fit/validation RMS) odak uzaklığı 33 piksel oynarken
0.05 px içinde kalıyor — **önemli olan hatayı hiç görmüyor.**

Bu, bu projede aynı desenin **üçüncü** görünüşü: (1) tahta saçılımı 9 mm'lik X
hatasında 1.04 mm gösteriyordu, (2) fronto-paralel dejenerasyonda reprojeksiyon
0.671 px'te kalıp 1.77°'lik poz hatasını gizliyordu, (3) şimdi kalibrasyon RMS'i
0.54°'lik poz belirsizliğini gizliyor.

**Hedefe etkisi:**

| model | σ_dönme | p90 |
| --- | ---: | ---: |
| yalnız ölçülen algı tabanı | 0.055° | 0.63 mm — geçer |
| + kalibrasyon kanalı | 0.450–0.540° | **4.39 mm — kalır** |
| + kanalın yarısı alınsa | 0.230° | 2.25 mm — kalır |

**Bugün erken saatte yazdığım "hedef geçildi" hükmü geçersizdir.** Dedektör
kazancı gerçek (σ_dönme 0.143 → 0.055) ama bağlayıcı kısıt artık orada değil.

### Bunun anlamı ve sıradaki iş

fx'in %6.6 oynaması, odak uzaklığının bu veriyle **kısıtlanmadığını** söylüyor.
Sebebi geometrik: odak uzaklığı ile mesafe birbirinin yerine geçebilir, ve bunu
ayıran şey farklı MESAFELERDEN ve GÜÇLÜ EĞİMLERDEN alınmış görüntülerdir.
Bugünkü yakalama kadrajın 3×3 kapsamasını doldurmaya odaklandı — araç onu
ölçüyor — ama derinlik/eğim çeşitliliğini ölçmüyor ve biz de gözetmedik.

- Kalibrasyon yakalaması **mesafe ve eğim çeşitliliğine** göre yeniden yapılmalı;
  kapsama haritası tek başına yeterli bir ölçüt değil. **Araçtaki eksik
  2026-08-26'da kapatıldı:** `capture_calib_images.py` artık ölçek ve kısalma
  oranından mesafe/eğim kutularını ölçüyor ve tamamlanma kapısına koyuyor.
  Yakalamanın kendisi hâlâ kamera istiyor.
- Kabul kapısı reprojeksiyon değil **poz kararlılığı** olmalı:
  `calibration_stability.py` bunu ölçüyor ve `--sigma-deg` ile kapıyı uyguluyor
  (bayrak 2026-08-26'da gerçekten eklendi; o tarihe kadar yalnız belgede vardı,
  kapı kalırsa süreç non-zero döner).
- Aday `imx219_640x480_20260818_v2.yaml` **etkinleştirilmedi**. Eskisinden %9 iyi
  olması, ikisinin de 0.54°'lik belirsizlik bulutu içinde olmasını değiştirmiyor.
