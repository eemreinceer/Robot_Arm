# `rendered_pdf_pages/` takipten çıkarıldı (2026-08-26, #12)

Bu dizindeki 8 PNG (`page-01…08.png`, ~1.3 MB) raporun **render edilmiş
sayfa görüntüleriydi** — kaynak belgeden türetilmiş ara ürün, kendi başına
kanıt değil.

Kaynak belge repoda duruyor:
`reports/final_rapor/6DOF_Robotik_Kol_Proje_Raporu.docx`

Sayfa görüntüsü gerekiyorsa kaynaktan yeniden üretilir:

```bash
libreoffice --headless --convert-to pdf 6DOF_Robotik_Kol_Proje_Raporu.docx
pdftoppm -png -r 150 6DOF_Robotik_Kol_Proje_Raporu.pdf page
```

Raporun kaynak diyagramları (`assets/*.png`) takipte KALDI: bunlar render
çıktısı değil, belgenin girdisi.
