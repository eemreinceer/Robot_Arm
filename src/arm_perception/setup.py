from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = "arm_perception"


def install_tree(source, destination):
    """Return setuptools data-file rows while preserving the web build tree."""
    rows = []
    root = Path(source)
    if not root.is_dir():
        return rows
    for file_path in sorted(root.rglob("*")):
        if file_path.is_file():
            relative_parent = file_path.parent.relative_to(root)
            rows.append((str(Path(destination) / relative_parent), [str(file_path)]))
    return rows

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/deploy/pi5", glob("deploy/pi5/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*.pt")),
        ("share/" + package_name + "/models/sorting_bins/bin_red", glob("models/sorting_bins/bin_red/*")),
        ("share/" + package_name + "/models/sorting_bins/bin_yellow", glob("models/sorting_bins/bin_yellow/*")),
        ("share/" + package_name + "/models/sorting_bins/bin_blue", glob("models/sorting_bins/bin_blue/*")),
        ("share/" + package_name + "/models/robot_arm_sorting_bins/bin_red", glob("models/robot_arm_sorting_bins/bin_red/*")),
        ("share/" + package_name + "/models/robot_arm_sorting_bins/bin_yellow", glob("models/robot_arm_sorting_bins/bin_yellow/*")),
        ("share/" + package_name + "/models/robot_arm_sorting_bins/bin_blue", glob("models/robot_arm_sorting_bins/bin_blue/*")),
        ("share/" + package_name + "/models/robot_arm_objects/red_box", glob("models/robot_arm_objects/red_box/*")),
        ("share/" + package_name + "/models/robot_arm_objects/yellow_cylinder", glob("models/robot_arm_objects/yellow_cylinder/*")),
        ("share/" + package_name + "/models/robot_arm_objects/blue_cube", glob("models/robot_arm_objects/blue_cube/*")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py")),
    ] + install_tree(
        "web/robot-arm-console/dist", "share/arm_perception/web/robot-arm-console/dist"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Emre",
    maintainer_email="inceer22@gmail.com",
    description="YOLO RGB-D and RGB planar pose estimation with autonomous pick integration.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = arm_perception.yolo_detector_node:main",
            "autonomous_pick_node = arm_perception.autonomous_pick_node:main",
            "capture_dataset = arm_perception.capture_dataset:main",
            "train_yolo = arm_perception.train_yolo:main",
            "validate_dataset_bboxes = arm_perception.validate_dataset_bboxes:main",
            "demo_sorting = arm_perception.sorting_scene:main",
            "acceptance_check = arm_perception.acceptance_check:main",
            "csi_camera_node = arm_perception.csi_camera_node:main",
            "rgb_planar_detector_node = arm_perception.rgb_planar_detector_node:main",
            "vision_encoder_node = arm_perception.vision_encoder_node:main",
            "mjpeg_bridge = arm_perception.mjpeg_bridge:main",
            "web_console = arm_perception.web_console:main",
        ],
    },
)
