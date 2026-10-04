# Desteklenen Ortam ve Sürüm Matrisi

Bu belge geliştirme ve hedef ortam sözleşmesini kaydeder. Fiziksel kabul veya
release bildirimi değildir.

## Geliştirme bilgisayarı

| Bileşen | Desteklenen / doğrulanan değer | Durum |
| --- | --- | --- |
| İşletim sistemi | Ubuntu 24.04 LTS | Aktif geliştirme hostu |
| ROS 2 | Jazzy | Kanonik host dağıtımı |
| Python | 3.12 | Host statik test ve ROS Python paketleri |
| PlatformIO Core | 6.1.19 | Yerel native/firmware doğrulamasında ölçüldü |
| ShellCheck | 0.9.0 | Yerel shell tabanında ölçüldü |

## Raspberry Pi 5 hedefi (aktif)

2026-08-26'da cihaz üzerinde ölçüldü (`ssh pi5`).

| Bileşen | Ölçülen değer | Not |
| --- | --- | --- |
| İşletim sistemi | Ubuntu 24.04.4 LTS | Host ile aynı taban |
| Çekirdek / mimari | 6.8.0-1061-raspi / `aarch64` | Raspberry Pi çekirdeği |
| ROS 2 | **Jazzy** | Nano'nun aksine host ile **aynı** dağıtım; ayrı konteyner gerekmiyor |
| Python | 3.12.3 | Host ile aynı |
| OpenCV | 4.6.0 | Kamera node'unun GStreamer yolu bunu kullanıyor |
| GStreamer | 1.24.2 | `libcamerasrc` üzerinden IMX219 |
| libcamera (apt) | 0.2.0 | **Kullanılmıyor** — kamera node'u ile uyumsuz |
| libcamera (yerel derleme) | **0.7.2+rpt20260817** | `/usr/local/lib/aarch64-linux-gnu`, unit içinde `GST_PLUGIN_PATH` ile açıkça seçiliyor |
| colcon | kurulu | Cihaz üzerinde build mümkün |

Kritik nokta: apt'nin libcamera 0.2.0'ı ile projenin derlediği 0.7.2 aynı
sistemde yan yana duruyor. `robot-arm-camera.service` doğru olanı seçmek için
`GST_PLUGIN_PATH` değişkenini unit içinde veriyor; bu satır kaldırılırsa
kamera sessizce yanlış eklentiyle açılmaya çalışır.

Deploy sözleşmesi ve systemd unit kopyaları
[`deploy/pi5/README.md`](../deploy/pi5/README.md) içindedir.

## Jetson Nano hedefi (EMEKLİ)

> Nano 2026-08 itibarıyla emekli edildi; aktif hedef Raspberry Pi 5'tir.
> Aşağıdaki tablo tarihsel kayıttır, yeni iş için taban alınmamalıdır.

| Bileşen | Doğrulanmış değer | Not |
| --- | --- | --- |
| JetPack/L4T ailesi | JetPack 4.x uyumlu yüzey | Nano uyumluluğu korunmalı |
| ROS 2 | Humble, konteyner | Jazzy install ağacı burada source edilmez |
| Host Python | 3.6.9 | Nano'ya doğrudan giden scriptler bu grameri korur |
| CUDA | 10.2.300 | 2026-07 ölçümü |
| TensorRT | 8.2.1 / Python 8.2.1.8 | Hedef cihaz ölçümü |
| Ultralytics export hostu | 8.4.75 | Laptop PT → ONNX adımı |
| ONNX | 1.22.0, opset 12 | Nano TensorRT yolu için doğrulandı |

Ayrıntılı Nano inference kanıtı
[`reports/nano_e2b_tensorrt.md`](../reports/nano_e2b_tensorrt.md) ve deploy
sözleşmesi [`deploy/nano/README.md`](../deploy/nano/README.md) içindedir.

## Firmware hedefleri

| Ortam | Hedef | Framework | Doğrulama |
| --- | --- | --- | --- |
| `firmware/stm32_servo_ctrl` | `bluepill_f103c8` | Arduino / PlatformIO | Cihaz build + `native` protokol testleri |
| `firmware/esp32_servo_ctrl` | `esp32dev` | Arduino / PlatformIO | Cihaz build + `native` hareket/protokol testleri |

PlatformIO platform paketleri `platformio.ini` içinde henüz kesin sürüme
sabitlenmemiştir. Bu, tekrarlanabilir release öncesinde kapatılması gereken
bilinçli bir boşluktur; CI tabanı şimdilik PlatformIO Core 6.1.19'u sabitler.

## Model ve release durumu

- Runtime modellerinin hash ve provenans durumu
  [`repository_artifact_inventory.md`](repository_artifact_inventory.md)
  dosyasında izlenir.
- Projenin kullanıcı tarafından atanmış bir semantik release sürümü yoktur;
  bu nedenle tahmini bir `VERSION` değeri eklenmemiştir.
- Repository portfolio evaluation için "all rights reserved" koşullarıyla
  yayınlanır; ayrıntılar kök [`LICENSE`](../LICENSE) dosyasındadır.

Bu matriste değişiklik yapılırken ölçüm tarihi, hedef cihaz ve üreten araç
sürümü birlikte güncellenmelidir.
