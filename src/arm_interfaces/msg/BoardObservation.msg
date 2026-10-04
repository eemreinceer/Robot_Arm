# Tek bir kameradan tek bir KARE için atomik tahta gözlemi.
#
# NEDEN VAR: pozu, tespit bayrağını ve reprojeksiyon hatasını üç AYRI topic'ten
# yayınlamak, tüketiciyi bunları kare bazında eşleştiremeyeceği bir duruma
# sokuyordu. `detected` ve `reprojection_px` stamp'siz olduğu için hangi kareye
# ait oldukları bilinemiyordu. 2026-07-31'de ölçüldü: zaman penceresiyle
# eşleştirme YETERLİ DEĞİL — yeni bir poz, kendi diagnostic'leri henüz
# yayınlanmadan geldiğinde ÖNCEKİ karelerin iyi değerleriyle kabul edilebiliyor.
#
# Bu mesaj kabul için gereken her şeyi tek bir stamp altında taşır. Tüketici
# bir örneği kabul ederken artık hiçbir şeyi varsaymaz.
#
# KURAL: `header.stamp` gözlemin türetildiği GÖRÜNTÜNÜN stamp'idir --
# yayınlama anı değil. Aynı görüntüden iki mesaj çıkarsa aynı stamp'i taşırlar.

std_msgs/Header header

# Tahta bu karede çözüldü mü. false ise aşağıdaki poz ve metrikler ANLAMSIZDIR
# ve tüketici bunları okumamalıdır.
bool detected

# camera_optical_frame -> board. Yalnız `detected` true iken geçerli.
geometry_msgs/Pose pose

# RMS reprojeksiyon hatası, piksel. Yalnız `detected` true iken geçerli.
# Bu karenin kendi değeridir; başka bir karenin değeri değildir.
float32 reprojection_px

# Poza katkı veren tespit edilmiş iç köşe sayısı. Kısmi tespitler bu sayıyla
# ayırt edilir; tam tahta beklenen değeri board_cols*board_rows'tur.
uint16 corners_detected
uint16 corners_expected

# Pozun hangi tahta geometrisiyle çözüldüğü. Üretici ile tüketicinin farklı
# tahta varsaymasını imkânsız kılar -- 2026-07-29'da bayat varsayılanlar tam
# olarak bu sınıfta bir arızaya yol açtı.
uint8 board_cols
uint8 board_rows
float32 square_size_mm
