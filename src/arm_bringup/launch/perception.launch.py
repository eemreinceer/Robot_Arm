"""
FAZ 4 — Tam otonom perception pick & place launch.

Tek komutta: Gazebo (+ sabit RGB-D kamera) + MoveIt2 + pick_place_node + RViz
(pick_and_place.launch.py'den) ÜZERİNE arm_perception 6DOF perception node'u.

Akış: kamera → YOLOv8 detection → depth/point-cloud 6DOF poz →
/detected_objects (base_link) → GetPickPose → /pick_and_place.

⚠️ Perception node ayrı `arm_perception` paketine bağlıdır:
   package='arm_perception', executable='perception_node' (KANONİK AD).
   Node hazır olana kadar kamera+sim'i test etmek için:
       ros2 launch arm_bringup perception.launch.py use_perception:=false

Tam otonom döngü (gör→çöz→tut→bırak) için autonomous_pick_node'u da başlat:
   ros2 launch arm_bringup perception.launch.py autonomous:=true

Sim sorting'de pick/place sonucu Gazebo fizik attach/drop davranışına bağlı
değildir. Robot action'ı çalışır; görsel sim nesnesi action başarısından sonra
deterministik set_pose sync ile hedef kutuya yazılabilir.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── arguments ────────────────────────────────────────────────────────────
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="true", description="Launch RViz"
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo clock"
    )
    use_perception_arg = DeclareLaunchArgument(
        "use_perception",
        default_value="true",
        description="Start arm_perception node (set false to test camera+sim only "
        "when perception_node is not installed)",
    )
    # Eğitilmiş YOLOv8 ağırlığı (gitignored). Boşsa node kendi varsayılanını kullanır.
    default_model = os.path.join(
        get_package_share_directory("arm_perception"), "models", "yolo_arm.pt"
    )
    perception_model_arg = DeclareLaunchArgument(
        "perception_model",
        default_value=default_model,
        description="Path to fine-tuned YOLOv8 weights (arm_perception/models/yolo_arm.pt)",
    )
    autonomous_arg = DeclareLaunchArgument(
        "autonomous",
        default_value="false",
        description="Start autonomous_pick_node (full gör→çöz→tut→bırak loop, spec 4.4). "
        "Requires perception_node publishing /detected_objects.",
    )
    # Faz 4B sınıf-bazlı sıralama: autonomous_pick_node'u sort_all moduna al.
    sort_all_arg = DeclareLaunchArgument(
        "sort_all",
        default_value="false",
        description="Class-based sorting: her nesneyi sınıfına ait kutuya yerleştir "
        "(autonomous:=true ile birlikte). sorting_bins_file ile kutu haritası verilir.",
    )
    sorting_bins_file_arg = DeclareLaunchArgument(
        "sorting_bins_file",
        default_value=os.path.join(
            get_package_share_directory("arm_perception"), "config", "sorting_bins.yaml"
        ),
        description="Sınıf→kutu poz haritası (BinSpec).",
    )
    # Hızlı test yolu için: true yapılırsa autonomous_pick_node nesneyi hemen
    # kutuya ışınlar ve taşıma fazlarını atlar. Normal sorting'de false kalır:
    # robot pick/place hareketlerini yapar; sim sonucu ayrı sync argümanıyla
    # deterministik olarak kutuya yazılır.
    fast_sort_arg = DeclareLaunchArgument(
        "fast_sort",
        default_value="false",
        description="true=nesneleri hemen kutuya ışınla (hızlı); false=robot pick/place hareketini çalıştır.",
    )
    simulation_sync_arg = DeclareLaunchArgument(
        "simulation_sync",
        default_value="true",
        description="Simulation-only: Gazebo nesne pozunu action sonucuna göre deterministik senkronla.",
    )
    simulation_post_place_sync_arg = DeclareLaunchArgument(
        "simulation_post_place_sync",
        default_value="true",
        description="Simulation-only: normal pick/place action başarısından sonra nesneyi hedef kutuya set_pose et.",
    )
    exact_pose_arg = DeclareLaunchArgument(
        "exact_pose",
        default_value="false",
        description="Simulation-only: true=Gazebo model pose ile pick/place hedefini override et.",
    )
    # Sorting/perception kendi temiz sahnesini kullanır (Faz-1 world'den ayrı).
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="sorting.world",
        description="Sorting/perception sahne world dosyası (Faz-1: pick_and_place.world).",
    )

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    use_perception = LaunchConfiguration("use_perception")
    perception_model = LaunchConfiguration("perception_model")
    autonomous = LaunchConfiguration("autonomous")
    sort_all = LaunchConfiguration("sort_all")
    sorting_bins_file = LaunchConfiguration("sorting_bins_file")
    fast_sort = LaunchConfiguration("fast_sort")
    simulation_sync = LaunchConfiguration("simulation_sync")
    simulation_post_place_sync = LaunchConfiguration("simulation_post_place_sync")
    exact_pose = LaunchConfiguration("exact_pose")

    # ── base system: gazebo + camera + move_group + pick_place_node + rviz ────
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_bringup"), "launch", "pick_and_place.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "world": world,
        }.items(),
    )

    # ── kinematics node — /ik_solve + /fk_solve (sorting sahne ön-kontrolü kullanır) ─
    kinematics_node = Node(
        package="arm_kinematics",
        executable="kinematics_node",
        name="kinematics_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── perception node (Faz 4, spec 4.3) — delayed until camera + TF are up ──
    perception_node = TimerAction(
        period=22.0,
        actions=[
            Node(
                package="arm_perception",
                executable="perception_node",
                name="perception_node",
                output="screen",
                condition=IfCondition(use_perception),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "model_path": perception_model,
                        # KİLİTLİ kontratlar (spec):
                        "image_topic": "/camera/image",
                        "camera_info_topic": "/camera/camera_info",
                        "points_topic": "/camera/points",
                        "detections_topic": "/detected_objects",
                        "target_frame": "base_link",
                    }
                ],
            )
        ],
    )

    # ── autonomous pick loop (Faz 4, spec 4.4) — after perception is publishing ─
    autonomous_node = TimerAction(
        period=26.0,
        actions=[
            Node(
                package="arm_perception",
                executable="autonomous_pick_node",
                name="autonomous_pick_node",
                output="screen",
                condition=IfCondition(autonomous),
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        # KİLİTLİ kontratlar (spec) — node defaultlarıyla aynı, açıkça pinle:
                        "detections_topic": "/detected_objects",
                        "pick_pose_service": "/get_pick_pose",
                        "pick_action": "/pick_and_place",
                        # Faz 4B sıralama wiring (yoksa sort_all sessizce yutuluyordu):
                        "sort_all": sort_all,
                        "sorting_bins_file": sorting_bins_file,
                        "simulation_fast_sort_enabled": fast_sort,
                        "simulation_sync_enabled": simulation_sync,
                        "simulation_post_place_sync_enabled": simulation_post_place_sync,
                        "simulation_exact_pose_enabled": exact_pose,
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            use_rviz_arg,
            use_sim_time_arg,
            world_arg,
            use_perception_arg,
            perception_model_arg,
            autonomous_arg,
            sort_all_arg,
            sorting_bins_file_arg,
            fast_sort_arg,
            simulation_sync_arg,
            simulation_post_place_sync_arg,
            exact_pose_arg,
            base_launch,
            kinematics_node,
            perception_node,
            autonomous_node,
        ]
    )
