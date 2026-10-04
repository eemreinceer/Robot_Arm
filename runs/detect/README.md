# `runs/detect/` — YOLO doğrulama çıktısı

Bu dizinde tutulan görseller (`val/*.png`, `val/*.jpg`) 2026-06-03'te
`0c0cee1` ile eklenen bir YOLO doğrulama koşusunun **çıktısıydı**: PR/F1/P/R
eğrileri, karışıklık matrisleri ve batch tahmin önizlemeleri.

2026-08-26'da takipten çıkarıldılar (#12). Gerekçe: yeniden üretilebilir
çıktılar, repodaki hiçbir kod/belge onlara atıf vermiyordu ve ~2 MB yer
kaplıyorlardı. Dosyalar diskte duruyor, yalnız git takibi bırakıldı.

## Yeniden üretmek için

```bash
python3 -m arm_perception.train_yolo --help   # eğitim/doğrulama parametreleri
```

Doğrulama, yerel olarak provision edilen
`src/arm_perception/models/yolo_arm.pt` ağırlığı ve ilgili veri seti ile
tekrarlanır; ağırlık public repoda dağıtılmaz. Ultralytics çıktıyı yine
`runs/detect/` altına yazar.

## Sayısal sonuçlar nerede

Bu görsellerin yanında `results.csv` benzeri bir metrik dosyası **yoktu**;
sayısal kabul değerleri PNG'lerden çıkarılamaz. Faz 4 algı kabul ölçümleri
`reports/` altındaki teknik raporlarda kayıtlıdır.
