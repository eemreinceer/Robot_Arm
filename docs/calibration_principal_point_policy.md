# Principal-point politikası: varyans ucuz, bias pahalı (#16)

Bu belge, `scripts/calibration_stability.py` ile üretilen
`runs/calib_stability_20260826.json` artefaktının okunuşudur. Artefakt
makine-okunur kanıttır; buradaki metin ondan çıkarılan hükümdür.

## Koşu kimliği

| | |
| --- | --- |
| Fit verisi | `calib_pi5_fit_merged_20260821` — 65 PNG, **64 algılandı** |
| Doğrulama verisi | `calib_pi5_val_20260821` — 16 PNG, 16 algılandı |
| Manifest (sha256 listesinin sha256'sı) | fit `b108fee85ed13193`, val `0ae8fbf37a6a3707` |
| Örnekleme | `subsample`, 12 çekiliş, alt küme 20, seed `20260826` |
| Tahta | 6×9, 27.5 mm |
| Üç aday | `free`, `image_center` (320/240), `pixel_grid_center` (319.5/239.5) |

Üç aday **tek çağrıda, aynı veri / seed / örnekleme** ile koşuldu.

## Ölçüm

| Politika | Belirsizlik (varyans) medyan | p90 | free'den kayma (bias) medyan | Doğrulama reproj. |
| --- | ---: | ---: | ---: | ---: |
| `free` | 0.4403° | 0.8565° | 0 (referans) | 0.1254 px |
| `image_center` | **0.2749°** | 0.6364° | **0.4613°** | 0.1265 px |
| `pixel_grid_center` | **0.2745°** | 0.6354° | **0.5093°** | 0.1265 px |

## Hüküm: takas kötü

Principal point'i sabitlemek belirsizliği medyanda **0.165°** düşürüyor, ama
karşılığında **0.46–0.51°** sistematik kayma getiriyor. Yani ödenen bedel,
alınan kazancın **yaklaşık 2.8 katı**.

Bu takas ancak sabitlenen nokta *doğruysa* kabul edilebilir. Veri onu
söylemiyor:

- `free` çekilişlerinde `cx` **321.03 – 327.72** aralığında; 12 çekilişin
  **hiçbiri** 320.0'ı ya da 319.5'i içermiyor.
- Canlı Pi `/camera/camera_info` da `cx=324.12` diyor.

Yani görüntü merkezine sabitlemek zararsız bir düzenlileştirme değil; modeli,
verinin işaret ettiği yerden **her çekilişte** uzaklaştırıyor.

## Kabul metriği yine önemli hatayı görmüyor

| Politika | Fit RMS medyan | Doğrulama ortalama |
| --- | ---: | ---: |
| `free` | 0.3084 px | 0.1254 px |
| `image_center` | 0.3112 px | 0.1265 px |
| `pixel_grid_center` | 0.3117 px | 0.1265 px |

Poz 0.5° kayarken reprojeksiyon **0.003 px** oynuyor. Bu, bu projede aynı
desenin **dördüncü** görünüşü — öncekiler
`docs/hand_eye_pose_set_requirement.md` §"HÜKÜM GERİ ALINDI" içinde listeli.
Kabul kapısı reprojeksiyon olamaz; poz kararlılığı olmalıdır.

## Açık kalan veri boşluğu

Araç iki uyarı bastı ve ikisi de gerçek:

```
fit metadata accepted=45, PNG count=65; merged dataset source manifest is required
fit metadata missing pipeline identity: camera_identifier, backend, pixel_format, libcamera_version
```

`calib_pi5_fit_merged_20260821` birleştirilmiş bir kümedir: `capture_meta.json`
tek bir oturumun `accepted=45` sayısını taşıyor ama dizinde 65 PNG var. Hangi
karenin hangi çekim oturumundan geldiği **kayıtlı değil**. #16'nın "fit veri
manifesti kaynak capture oturumlarını doğrular" kriteri bu yüzden henüz
kapanmadı; kare sayısı ve hash'ler artefaktta, oturum kökeni değil.

## Ne yapılmadı

Aktif intrinsics YAML'ı ve Pi deploy'u **değiştirilmedi** (#16 kriteri).
Kamera, robot, UART ve servo rayı kullanılmadı; bu tamamen çevrimdışı bir
analizdir. Algı gürültü tabanı karşılaştırması da yapılmadı: mevcut
`runs/hand_eye/noise_floor*.json` dosyaları `{meta, samples}` şemasında ve
aracın beklediği `angular_deg` özetini taşımıyor; onu üreten
`measure_perception_floor.py` canlı kamera istiyor.
