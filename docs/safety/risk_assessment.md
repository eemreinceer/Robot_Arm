# Risk Analizi — 6DOF Collaborative Robot Arm

**Durum:** TASLAK v0.1 (2026-05-31) — proje sahibiyle birlikte hazırlanıyor
**Yöntem:** ISO 12100:2010 (risk değerlendirme süreci) + ISO 13849-1 (PLr) + ISO/TS 15066:2016 (cobot biyomekanik limitler)
**Kapsam kararı (kullanıcı, 2026-05-31, sade soru-cevap ile doğrulandı):**
- İnsanlar robotun **kol mesafesinde** (temas mümkün → full-contact)
- Nominal payload **~1.0 kg** (kullanıcı bana bıraktı; tork limitinden türetildi)
- Robot nesneleri **insanın üzerinden** geçirebilir → düşen-nesne tehlikesi **F2 (sık)**, H3 → **PLd**
- Acil durdurma: **hem fiziksel düğme hem otomatik (çarpışma-sezme)** olacak (kullanıcı kararı)

> ⚠️ Bu bir mühendislik taslağıdır, sertifika belgesi değildir. CE işareti (Makine Yönetmeliği
> 2006/42/EC → Makine Tüzüğü 2023/1230) bir notified body + tam teknik dosya gerektirir. Bu belge
> teknik dosyanın risk-analizi bölümünün taslağıdır.

---

## 1. Makine Sınırları (ISO 12100 §5.3)

| Sınır | Değer / Açıklama |
|---|---|
| **Kullanım** | 6 serbestlik dereceli manipülatör; genel amaçlı al-taşı-bırak + hafif kuvvet görevleri |
| **Operasyon modları** | INDUSTRIAL (insan yok, tam hız/tork) ↔ COLLABORATIVE (PFL+SMS aktif) |
| **Payload** | Nominal 1.0 kg (tam erişim, %40 tork rezervi). ~2 kg kısa erişimde. Kaynak: joint_2 limiti 30 Nm |
| **Erişim** | ~0.785 m (omuzdan uç efektöre, yatay) |
| **Hız sınırı** | Joint vel limitleri 1.0–2.0 rad/s (URDF). COLLABORATIVE modda ölçeklenecek |
| **İnsan yakınlığı** | Temas mümkün (aynı çalışma alanı, eş zamanlı) |
| **Çevre** | İç mekan, sabit montaj (base_link tezgâha z=0.6'da çakılı), kuru |
| **Ömür/maruziyet** | Sürekli işbirlikli çalışma varsayımı → F2 (sık/sürekli maruziyet) |

**Açık noktalar (kullanıcı doğrulasın):** gerçek montaj yüksekliği, uç efektör tipi (gripper sabit mi, takım değişir mi), gerçek güç kaynağı (24V mı, şebeke mi).

---

## 2. PLr atama yöntemi (ISO 13849-1 risk grafiği)

Her güvenlik fonksiyonu için: **S** (yaralanma şiddeti) × **F** (maruziyet sıklığı) × **P** (kaçınma imkânı)

- S1 hafif/geri-dönüşlü · **S2 ciddi/geri-dönüşsüz**
- F1 seyrek · **F2 sık/sürekli** (full-contact cobot → çoğunlukla F2)
- P1 kaçınma mümkün · **P2 neredeyse imkânsız**

Full-contact + sürekli maruziyet seçtiğimiz için çoğu fonksiyon **PLd** bandına oturuyor.

---

## 3. Tehlike Kaydı (ISO 12100 §5.4)

| # | Tehlike | Tip | Senaryo | S·F·P | **PLr** | Önlem (Faz 6 alt-fazı) | Kalan risk |
|---|---|---|---|---|---|---|---|
| H1 | **Geçici çarpma (transient impact)** | Mekanik | Hareketli kol insan vücut bölgesine vurur | S2·F2·P2 | **PLd** | PFL kuvvet limitleme (6.4): çarpma kuvveti < ISO/TS 15066 Tablo A.2 | Düşük |
| H2 | **Sıkıştırma/kapma (quasi-static clamping)** | Mekanik | İnsan kol ile sabit yapı arasında sıkışır — kuvvet boşalamaz, en kötü durum | S2·F2·P2 | **PLd** | PFL quasi-static limit + sıkışma-noktası geometrisi + çarpışma durdurma (6.3) | Düşük |
| H3 | **Düşen/fırlayan payload** | Mekanik | Güç kaybı/gripper arızasında 1 kg nesne düşer — nesne insan üzerinden geçebildiği için maruziyet sık (F2) | S2·F2·P2 | **PLd** | Gripper kendinden kilitli / güç-kesintisinde tutma + hız sınırı + mümkünse rota insan üstünden geçmesin | Orta→Düşük |
| H4 | **Beklenmeyen hareket / kaçış (runaway)** | Kontrol | Kontrolcü/yazılım hatası → hızlı/yanlış hareket | S2·F2·P2 | **PLd** | Hız & tork izleme (6.2 Safety Monitor 200Hz), watchdog, limit denetimi | Düşük |
| H5 | **Mod karışıklığı** | Sistemik | INDUSTRIAL mod insan varken aktif kalır → tam hız | S2·F2·P2 | **PLd** | OperationMode güvenli geçiş + insan algılandığında zorunlu COLLABORATIVE (6.7) | Düşük |
| H6 | **Acil durdurma başarısızlığı** | Sistemik | E-stop'a basılır ama robot durmaz | S2·F2·P1 | **PLd** | Latched kategori-1 E-stop (6.6), ISO 13850 | Düşük |
| H7 | **Uç efektör keskin kenar/takım** | Mekanik | Temas anında kesik/delinme | S2·F2·P2 | **PLc** | Tasarımla giderme (yuvarlatma) + PFL | Düşük |
| H8 | **Elektrik çarpması** | Elektrik | (Gerçek donanım) yalıtım arızası | S2·F1·P2 | TBD | IEC 60204-1 — donanım fazında | — (sim dışı) |

---

## 4. ISO/TS 15066 biyomekanik limitler (PFL hedefi)

Full-contact seçtiğimiz için H1/H2'nin kabul kriteri bu tablodur. Örnek vücut-bölge limitleri (quasi-static):

| Vücut bölgesi | Maks. kuvvet (quasi-static) | Geçici (transient) ≈ |
|---|---|---|
| El / parmak | ~140 N | ~280 N |
| El sırtı | ~140 N | ~280 N |
| Alt kol | ~160 N | ~320 N |
| Üst kol / omuz | ~210 N | ~420 N |
| Yüz / kafatası | **engellenmeli** (temas yasak bölge) | — |

> PFL kontrolcüsü (6.4) bu limitleri kartezyen kuvvet tavanı olarak uygular. "Yüz/kafatası temas yasağı"
> çalışma alanı/yükseklik tasarımıyla garanti edilir (robot bu bölgeye erişemesin).

---

## 5. Sonraki adımlar

- [ ] Bölüm 1 "açık noktalar"ı kullanıcı ile netleştir
- [ ] H1/H2 için sim'de gerçek çarpma kuvvetini ölç (effort → kartezyen kuvvet dönüşümü) ve Tablo A.2 ile karşılaştır
- [ ] Her PLd fonksiyon için ISO 13849-1 mimari (kategori, MTTFd, DC) taslağı
- [ ] Gerçek donanım alt-fazlarını bu tehlikelere bağlı ayrı bir safety spec'e dök
