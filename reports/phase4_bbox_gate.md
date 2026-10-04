# Faz 4 BBox Kabul Kapısı Raporu

**Tarih:** 2026-05-31  
**Durum:** Root mount düzeltmesi alındı; 2D bbox ve kanonik 6DOF GT ön koşulu doğrulandı. YOLO eğitimi kullanıcı onayı olmadan başlatılmadı.

## Alınan Root Mount Düzeltmesi

İlgili düzeltme commitleri:

- `9432506 fix(arm_description): anchor robot base to world — stop free-fall in Gazebo`
- `0d7b486` — robot base anchored, gate re-run

İlgili paketler yeniden derlendi:

```bash
colcon build --symlink-install --packages-select arm_description arm_gazebo arm_perception
```

Canlı doğrulama:

```text
gz model -m 6dof_arm --pose
XYZ = [0.000000, 0.000000, 0.000000]
RPY = [0.000000, -0.000000, 0.000000]

world -> base_link
Translation = [0.000, 0.000, 0.600]
Rotation = [0.000, 0.000, 0.000, 1.000]

base_link -> camera_optical_frame
Translation = [0.400, 0.000, 0.900]
```

Robot root pozu 20 sahnelik capture sonrasında da değişmedi. Üç controller aktifti.

## BBox Gate Koşumu

Gate temiz geçici dizinde, `--append` kullanılmadan çalıştırıldı:

```bash
ros2 run arm_perception capture_dataset \
  --output /tmp/arm_perception_bbox_gate_root_fixed \
  --samples 20 --settle-seconds 0.7 --fresh-frames 3 --max-attempts 10

ros2 run arm_perception validate_dataset_bboxes \
  --dataset /tmp/arm_perception_bbox_gate_root_fixed \
  --samples 20 --seed 42 --min-iou 0.5 \
  --output reports/faz4_dataset_bug/bbox_gate_20.jpg
```

Sonuç:

```text
samples=20 min_iou=0.843011 mean_iou=0.929943 pass_gt_0.5=20/20
```

Bbox boyutları `29.6-46.7 px` aralığında. Bu değerler beklenen yaklaşık `28-69 px` bandında; eski yaklaşık `145 px` sapması kapandı.

Overlay montage: `reports/faz4_dataset_bug/bbox_gate_20.jpg`

## Sonraki Adım

1. Eski bozuk dataset kullanılmadan, `--append` vermeden temiz tam dataset üretilecek.
2. Tam dataset üzerinde aynı bbox gate tekrar çalıştırılacak.
3. Kullanıcı ayrıca onay verdikten sonra final YOLO eğitimi başlayacak.
