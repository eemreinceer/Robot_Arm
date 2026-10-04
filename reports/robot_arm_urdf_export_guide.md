# Robot Arm Kol — SW2URDF Yeniden Export Rehberi (7 eklem, hareketli gripper)

Tarih: 2026-07-06 · Talep: kullanıcı

## Amaç
Gripper'ın iki parmağı GERÇEKTEN hareket etsin. Eski export gripper'ı tek blok
(`Link_6`) yaptı → parmaklar kıpırdamıyor. Yeni export'ta gripper tabanı Link_5'e
dahil, iki parmak ayrı hareketli link.

## Eklem haritası (6 servo → 7 revolute eklem)
| Eklem | Tip | Servo | Açıklama |
|---|---|---|---|
| joint_1 | revolute | servo 1 | kol |
| joint_2 | revolute | servo 2 | kol |
| joint_3 | revolute | servo 3 | kol |
| joint_4 | revolute | servo 4 | kol |
| joint_5 | revolute | servo 5 | kol (wrist) |
| joint_6 | revolute | servo 6 | SOL parmak (gripper sürücü) |
| joint_6_mirror | revolute | — (dişli) | SAĞ parmak — servo YOK, joint_6'ya kenetli |

- **7 dönme ekseni + 7 koordinat sistemi** tanımlanacak (base_link kök CS'i ayrıca).
- Gripper **tabanı (dişli kutusu + servo yuvası) = Link_5'e dahil** (ayrı link değil).
- `joint_6_mirror`'ın servosu yok; dişli bağını ROS'ta ben mimic/2. sürücü olarak kuruyorum.

---

## FAZ A — SolidWorks'te referans geometri (exporter'dan ÖNCE)

- [ ] A1. Tam kol assembly'sini aç (aynı CAD → Link_1..5 birebir aynı çıkacak, güvenli).
- [ ] A2. **joint_1..joint_5** için: her kol servosunun dönme ekseninde bir **Reference Axis**
      (Insert → Reference Geometry → Axis) + eksen üzerinde bir **Coordinate System**
      (origin = eklem merkezi). Toplam 5 eksen + 5 CS.
- [ ] A3. **Gripper parmak pivotları** için: sol parmağın döndüğü pim/dişli ekseninde 1
      Reference Axis + 1 Coordinate System (joint_6); sağ parmak için ayrıca 1 Axis + 1 CS
      (joint_6_mirror). Toplam +2 eksen + 2 CS.
- [ ] A4. **Konvansiyon:** her koordinat sisteminin **Z ekseni = o eklemin dönme ekseni**
      olacak şekilde hizala (URDF standardı; SW2URDF referans ekseni ayrı da seçtirir ama
      Z=dönme en temizi).
- [ ] A5. Parça gruplarını netleştir: **gripper tabanı/dişli kutusu/servo yuvası → Link_5 ile
      aynı grup**; **sol parmak parçaları → finger_left**; **sağ parmak parçaları → finger_right**.
      (Dişliler hangi parmakla dönüyorsa o parmağa dahil et.)

## FAZ B — SW2URDF link ağacı

- [ ] B1. Tools → **Export as URDF** (SW2URDF eklentisi).
- [ ] B2. Link ağacını kur:
      `base_link → Link_1 → Link_2 → Link_3 → Link_4 → Link_5 → (finger_left, finger_right)`
- [ ] B3. **joint_1..joint_5:** her biri **revolute**, ilgili A2 koordinat sistemi + ekseni,
      limitleri (kol servosu aralığı; bilmiyorsan revolute + geniş limit ver, continuous DE olur).
- [ ] B4. **Link_5 komponentleri:** wrist parçaları **+ gripper tabanı/dişli kutusu** (parmaklar HARİÇ).
- [ ] B5. Link_5'e **iki child link** ekle:
      - **finger_left** → sol parmak parçaları. Joint adı `joint_6`, tip **revolute**, A3 sol CS/eksen,
        limit = parmağın tam açık↔tam kapalı açısı.
      - **finger_right** → sağ parmak parçaları. Joint adı `joint_6_mirror`, tip **revolute**, A3 sağ CS/eksen,
        aynı stroke (ters yön).
- [ ] B6. Joint eksen yönlerini doğrula (en sık hata burada): her joint ekseni = gerçek dönme ekseni.

## FAZ C — Export & teslim

- [ ] C1. Global birim **metre**, mesh format **STL**.
- [ ] C2. Preview ile eklem ağacını gözden geçir (7 eklem, isimler doğru mu).
- [ ] C3. **Export** → bir ROS paketi üretir: `meshes/` (link başına 1 STL: Link_1..5, finger_left,
      finger_right) + `urdf/*.urdf` (joint origin/axis/limit'lerle).
- [ ] C4. Klasörü repoya at (örn. `robot_arm_export_v2/`), yolunu proje sorumlusuna ver.

## Teslim sonrası entegrasyon
- Yeni mesh'leri `robot_arm_description/meshes/`'e alır; `robot_arm_body.xacro`'yu yeni link/joint
  origin/axis/limit'leriyle günceller (benim eklediğim geçici beyaz parmakları siler).
- `joint_6_mirror`: **gerçek** varyantta URDF `mimic(joint_6,-1)` (tek servo, dişli), **sim**
  varyantta 2. sürücü eklem (gz mimic desteklemiyor) — bu altyapı hazır.
- Controller (`robot_arm_gripper_controller`), SRDF gripper grubu/open-closed, kalibrasyon kanal
  haritasını yeni yapıya bağlar; sorting 6/6 regresyon + gz'de simetrik aç/kapa doğrular.

## Notlar
- **6 servo doğru; 7 eklem** çünkü joint_6 servosu iki parmağı dişliyle sürüyor (biri mimic).
- Eğer 6. servo aslında gripper'ı DÖNDÜREN bir wrist-roll ise (parmak aç/kapa değil), bana söyle —
  o zaman harita değişir. (A0 kararına göre joint_6 = gripper aç/kapa varsayıldı.)
- Kol linklerini değiştirme; sadece gripper'ı bölüyoruz. Aynı CAD olduğu için Link_1..5 aynı gelir.
