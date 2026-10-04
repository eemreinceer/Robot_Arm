# Dürüst Durum Raporu — Sıralama log'da "çalışıyor", gerçekte çalışmıyor (2026-06-06)

## Özet (TL;DR)

Terminal `DOĞRU KUTU` basıyor ama Gazebo görüntüsünde **tüm kutular boş**, nesneler masada/yerde.
İki bağımsız illüzyon üst üste biniyor:

1. **Grasp sahte** — fiziksel tutuş yok, nesne `gz set_pose` ile ışınlanarak gripper'ı takip
   etmeye çalışıyor; tutmuyor/yerleştirmiyor.
2. **Judge geçici pozu ölçüyor** — nesnenin XY'si bir AN için kutu üstünden geçince "correct"
   damgalıyor ve bir daha bakmıyor; nesnenin son (oturmuş) konumunu doğrulamıyor.

Sonuç: **hiçbir nesne fiziksel olarak sınıflandırılmıyor.** "DOĞRU" basan red'ler bile kutuda değil
(biri yerde).

## Kanıt

Kullanıcı görselleri (integ-23 sonu): red/yellow/blue kutuların **üçü de boş**; sarı silindir + mavi
küp masada (spawn bölgesinde); bir kırmızı küp robot tabanının yanında **yerde**. Terminal ise
red_box_00, red_box_01, blue_cube_01 için `DOĞRU KUTU` basmıştı.

## Kök neden 1 — grasp bir teleport, fizik değil (`src/arm_gazebo/scripts/sim_grasp_node.py`)

- Gripper kapanınca nesnenin offset'ini hesaplar, `ATTACH ... fixed-joint latched` basar —
  ama **gerçek bir joint KURULMAZ**.
- Nesneyi `gz service set_pose` ile ~8 Hz ışınlayarak gripper'ı takip ettirir (fizik değil).
- Gripper açılınca `DETACH released at <hesaplanan hedef>` basar — nesne oraya fiziksel ulaşsa da
  ulaşmasa da.
- Başarısızlık modları: (a) yavaş RTF'de `set_pose` çağrıları başarısız ("Host unreachable") →
  nesne hiç hareket etmez; (b) çalışsa bile ışınlama olduğu için bırakınca nesne kutuda oturmaz,
  düşer/sekip dışarı çıkar.
- Bu **bilinen** bir WIP kısıtı: `reports/sim_grasp_wip.md` + STATUS eski dürüstlük notu
  (*"5/5 = IŞINLAMA, fiziksel tutuş DEĞİL"*).

## Kök neden 2 — judge geçici pozu ölçüyor (`src/arm_tests/integration/test_real_grasp_sorting.py`)

- `wait_until_sorted` her nesnenin canlı gz pozunu yoklar ve XY'si bir kutu üstünde + z<0.75 olduğu
  **İLK AN** "correct" damgalayıp o nesneyi izlemeyi bırakır (`status != pending`).
- Teleport-takibi sırasında / kısa DETACH anında nesne bir an kutu üstünden geçer → judge o anı
  yakalar → "correct" kilitlenir → son oturmuş poz hiç kontrol edilmez.
- Yani anlık bir "üstünden uçma" bile başarı sayılır.

## Bu geceki çalışmanın dürüst değeri

Gerçek altyapı bug'ları bulundu ve düzeltildi (working tree, commit'siz) — AMA bunlar yalnızca
**sahte pipeline'ın daha uzağa koşmasını** sağladı, gerçek tutuş YARATMADI ("gerekli ama yeterli değil"):

1. `allowed_start_tolerance 0.1→0.5` (`arm_moveit_config/launch/move_group.launch.py`) —
   place-descent execution abort'ları (`start point deviates ... joint_3`) düzeldi.
2. `follow_rate 20→8 Hz` + `set_pose timeout 300→1500 ms` (`sim_grasp_node.py`) —
   gz-transport "Host unreachable" seli/donması düzeldi.
3. `joint_3` planlama limiti `-3.14→-3.30` (`arm_description/urdf/arm.urdf.xacro`; ros2_control
   command `<param min>` tutarlılık için de -3.30) — OMPL "start state out of bounds at joint_3"
   kilidi (eski correct=4 tavanı) düzeldi; döngü artık 6 nesneye de ilerliyor.

> Not: bu 3 fix gerçek tutuş geldiğinde GENUINELY faydalı olacak; ama tek başlarına sıralamayı
> çalıştırmazlar.

## Gerçek bloker

**Fiziksel tutuş yok.** Grasp gerçek olana kadar hiçbir nesne kutuya girmeyecek.

## GERÇEK fiziksel tutuş seçenekleri

| Seçenek | Nasıl | Artı | Eksi |
|---|---|---|---|
| **A. gz DetachableJoint** | gz-sim 8 yerleşik sistemi; kapanınca gripper↔nesne GERÇEK fixed-joint, açılınca kop (topic ile tetik) | Gerçek fizik tutuş; nesne fiziksel taşınır + kutuya düşer; teleport tamamen kalkar | Mimari: world/SDF wiring + spawn edilen modele dinamik attach + tetik mantığı; orta efor |
| **B. Friction grasp** | parmak/nesne sürtünme + tutma kuvveti + gripper PID tune | En gerçekçi, özel eklenti yok | gz'de tune'u en zor/kırılgan; küçük nesneler kayar |
| **C. Dürüst teleport demo** | teleport KALIR ama (1) son oturmuş poz doğrulanır, (2) "fiziksel tutuş değil, algı+yönlendirme demosu" olarak kabul edilir | Hızlı; kapsam konusunda dürüst | Hâlâ "gerçek" sıralama değil |

## Yaklaşımdan bağımsız ZORUNLU düzeltme

**Judge son oturmuş pozu doğrulamalı** (örn. kol bölgeden ayrıldıktan sonra N ardışık yoklamada
XY-kutuda VE z bin tabanı–ağzı aralığında stabil), geçici poz değil. Aksi halde HER yaklaşım
yanlış-pozitif raporlar.

## Öneri

**Seçenek A (DetachableJoint) + judge'ı oturmuş-poz doğrulamasına geçir.** Nesnelerin fiziksel
olarak kutuya girmesinin tek yolu bu. Plan:
1. Sim'e DetachableJoint wire et (gripper↔dinamik spawn nesne).
2. Grasp'ta attach / release'te detach tetikle (gripper joint eşiğiyle).
3. Judge'ı oturmuş-poz doğrulamasına çevir.
4. Sonra bu geceki 3 altyapı fix'i gerçekten işe yarar hale gelir → gerçek correct=6 ölçülür.

## Karar bekleniyor

Kullanıcı "önce rapor + plan" dedi. Yaklaşım (A/B/C) seçilince uygulamaya geçilecek.
Working tree'deki 3 altyapı fix'i şimdilik commit EDİLMEDİ (önce yön kararı).
