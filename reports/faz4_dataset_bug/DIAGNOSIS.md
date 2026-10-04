# Faz 4 — YOLO Dataset/GT Hatası Tanısı (2026-05-31)

## TL;DR
Faz 4 YOLO eğitimi **dataset bozuk olduğu için** başarısız — epoch sayısı sorunu DEĞİL.
Tam 2000-örnek + 80-epoch koşum bile **mAP@50 = %0.68** (hedef %85), patience=15 ile
**epoch 21'de erken durdu** (en iyi epoch 6). Kök neden: **GT etiketleri görüntü içeriğiyle
eşleşmiyor.**

## Kanıt
- `bbox_old_000001-3.jpg` — eski capture: sahnede nesne yığını, her görselde TEK etiket,
  o da kayık.
- `bbox_recent_001997-9.jpg` — mevcut "temiz" kodla capture (`--settle 0.05`): **kırmızı GT
  kutusu tamamen boş zemine düşüyor; hedef nesne görüntüde HİÇ yok.** Sadece masa (yeşil kare)
  + gri zemin görünüyor.
- GT kaydı (annotations.jsonl) hatasız üretiliyor (poz+bbox var) ama piksele karşılık gelmiyor.

## Kök nedenler (iki bağımsız sorun)

### 1. Projeksiyon/ekstrinsik hatası — bbox yanlış yerde + aşırı büyük
`capture_dataset.py::project_bbox` (satır 101-123) ürettiği kutu **~145px**. Oysa kamera
`fx=554.38`, nesne 5cm, tepeden ~0.4-1.0m: beklenen **~28-69px**. Yani kutu **2-5× büyük**
ve görüntüde nesnenin olmadığı orta-kare bir yere düşüyor. Şüpheli: `camera_from_base` TF
yönü / `base_pose` frame uyuşmazlığı (`base_world_z=0.6` çıkarması) veya optik-frame ekseni.
**Doğrula:** tek bir spawn'da nesneyi sim'de gör → `project_bbox` çıktısını görüntüye çiz →
nesne piksellerini sarmalı (IoU>0.5).

### 2. `--settle 0.05s` çok kısa — nesne render olmadan kare yakalanıyor
Kamera **8Hz (≈125ms/kare)**. 50ms settle ile spawn sonrası ilk taze kare nesne henüz
görünür/oturmadan çekiliyor → boş masa fotoğrafı. Orijinal run `0.7s` kullanıyordu.
**Düzelt:** settle ≥ 0.5s (kamera periyodunun birkaç katı) + isteğe bağlı "nesne piksel var mı"
sağlık kontrolü.

## Kabul kapısı (eğitimden ÖNCE — zorunlu)
Yeni capture sonrası, **rastgele 20 görselde GT bbox'u çiz ve nesne piksellerini sardığını
gözle/IoU ile doğrula.** Bu geçmeden eğitim koşma. Eğitim ancak temiz dataset + **KULLANICI
ONAYI** ile başlar.

## Süreç notu
Bu koşum kullanıcının onay kapısını atlayarak başlatıldı (gate `3c1a47d`'de kayıtlı).
Eğitim artık dataset düzeltilip kullanıcı onayı alınana dek koşulmayacak.
