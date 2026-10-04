# ROS 2 workspace map

The `src/` tree contains eight active packages and five preserved experiments.
Only the active set is part of the default `colcon` build and CI contract.

## Active packages

| Package | Responsibility |
| --- | --- |
| `arm_interfaces` | Project messages, services and actions |
| `robot_arm_description` | Active URDF/Xacro, meshes and controller configuration |
| `robot_arm_moveit_config` | Active MoveIt 2 planning configuration |
| `arm_hardware` | Bounded UART `ros2_control` interface |
| `arm_nodes` | Pick/place orchestration and measurement tools |
| `arm_perception` | Camera, calibration, detection and operator console |
| `arm_bringup` | Active simulation and real-hardware launch orchestration |
| `arm_tests` | Unit, integration and benchmark surfaces |

## Preserved legacy packages

`arm_description`, `arm_gazebo`, `arm_kinematics`, `arm_ml` and
`arm_moveit_config` are excluded with `COLCON_IGNORE`. They remain as evidence
of earlier simulation, IK and ML work; they must not be mistaken for the active
Robot Arm runtime. Their re-enablement contract and remaining limitations are
documented in [`LEGACY_6DOF.md`](LEGACY_6DOF.md).

Do not remove a `COLCON_IGNORE` marker in the active workspace merely to make a
legacy demo available. Build preserved experiments in an isolated checkout so
their dependency and launch assumptions cannot contaminate the active baseline.
