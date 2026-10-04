# Legacy 6DOF paketleri — varsayılan build dışında

**Tarih:** 2026-08-07 · **Karar:** proje sahibi

Faz 6'da hedef robot **robot_arm** küçük kola döndü. Aşağıdaki beş paket **eski
6DOF kola** aittir ve canlı robot_arm yolunda kullanılmaz. Her birinde bir
`COLCON_IGNORE` dosyası var, yani `colcon build` onları atlar.

| paket | son commit | ne |
| --- | --- | --- |
| `arm_kinematics` | 2026-06-06 | eski kolun FK/IK servisleri |
| `arm_moveit_config` | 2026-06-19 | eski kolun MoveIt yapılandırması |
| `arm_ml` | 2026-06-23 | DL-IK modeli; hiçbir launch'tan erişilmiyor |
| `arm_gazebo` | 2026-06-28 | eski sim dünyası + nesne modelleri |
| `arm_description` | 2026-06-29 | eski kolun URDF/xacro'su |

**Silinmediler.** Kullanıcı eski Gazebo pick & place sim'ini demo/tez için
saklamak istiyor; `COLCON_IGNORE` yalnızca varsayılan build'i sadeleştirir,
yeteneği yok etmez.

## Neden bu ayrım güvenli — ölçüldü (2026-08-07)

Canlı ve legacy yollar launch dosyası düzeyinde zaten ayrık:

| launch | yol | kullandığı paketler |
| --- | --- | --- |
| `robot_arm_perception.launch.py` | 🟢 canlı | arm_nodes, arm_perception, robot_arm_description, robot_arm_moveit_config |
| `real_hardware.launch.py` | 🟢 canlı | arm_nodes, robot_arm_description, robot_arm_moveit_config |
| `pick_and_place.launch.py` | 🔴 legacy | arm_description, arm_gazebo, arm_moveit_config, arm_nodes |
| `sim_only.launch.py` | 🔴 legacy | arm_description, arm_gazebo |
| `perception.launch.py` | 🔴 legacy | arm_kinematics, arm_perception |

`arm_perception` **canlı bir pakettir** ve `arm_gazebo`'ya bakan üç yeri var,
ama hiçbiri robot_arm yolunda çalışmıyor:

- `sorting_config.py` — `objects_package` varsayılanı `'arm_gazebo'`, ama bu
  yalnızca bir **string varsayılanı**; paket dizini ancak profil öyle derse
  çözülür. Robot Arm profili (`config/robot_arm_sorting_bins.yaml`) `arm_perception` diyor.
- `sorting_scene.py` — yalnız `demo_sorting` entry point'inden çağrılır.
- `capture_dataset.py` — yalnız `capture_dataset` entry point'inden çağrılır.

Robot Arm launch'ı sadece `perception_node` ve `autonomous_pick_node` çalıştırıyor;
ikisi de bu üç yola girmiyor.

`arm_tests/utils/gz_gt.py` de `arm_gazebo` modellerine bakar, ama yalnızca
`ARM_TESTS_STACK=1` ile açılan stack testlerinden çağrılır (bkz.
`src/arm_tests/conftest.py`).

## Legacy sim'i geri açmak

```bash
rm src/arm_{description,gazebo,moveit_config,kinematics,ml}/COLCON_IGNORE
colcon build --symlink-install
```

Sonra eski sim:

```bash
ros2 launch arm_bringup pick_and_place.launch.py
```

## Bilinen bakiye

- `arm_bringup/package.xml` hâlâ beş legacy paketi `depend` olarak listeliyor.
  Bu, ignore edilmiş paketler için colcon uyarısı üretebilir; build'i kırmıyor.
  Bağımlılıkları ayırmak, legacy launch'ları ayrı bir pakete taşımayı gerektirir
  — bu oturumda yapılmadı.
- `arm_tests/benchmarks/benchmark_ik_comparison.py` `arm_kinematics` ve
  `arm_ml`'e bakıyor; ignore ile bu benchmark koşmaz.
- Legacy benchmark raporları artık script konumundan türetilen repo-relative
  `src/arm_tests/benchmark_results/` dizinine yazılır; kullanıcıya özel sabit
  workspace yolu kullanılmaz.
