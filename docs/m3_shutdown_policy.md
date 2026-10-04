# M3 Mock Bringup Shutdown Policy

Source: retained shutdown investigation from the pre-portfolio engineering log
Decision date: 2026-08-25 (identity remeasured 2026-10-02)
Scope: ROS 2 Jazzy, `real_hardware.launch.py mock_serial:=true`, no robot or simulation

## 2026-08-25 revision — what changed and why

The first version of this policy accepted `move_group` exit `-15` and described
the root cause as a *probable* lifetime-order defect "without a complete native
backtrace". Both statements are now superseded by measurement.

`move_group` does not hang. It **segfaults** during teardown. The hang that was
observed — and the `-15` that followed it — was produced by this workstation's
`DEBUGINFOD_URLS` environment variable: `backward_ros`' crash handler tries to
symbolize the faulting stack through libdw, libdw calls out to
`https://debuginfod.ubuntu.com` over HTTPS, and that network call blocks the
already-crashed process indefinitely. Launch then escalated SIGINT to SIGTERM
and reported `-15`.

This is why every earlier launch-side experiment failed to move the needle: the
launch files were never involved. The corrected exception below is therefore
`-11`, not `-15`, and the test environment must pin `DEBUGINFOD_URLS=""`.

## Decision

The normal contract remains: every process launched by M3 exits with code `0`.
There is one temporary, visible exception for the exact installed MoveIt binary
identified below. The exception must never be implemented by omitting
`move_group` from the required-process list, skipping the test, or accepting any
arbitrary nonzero exit.

No `arm_bringup` timeout or shutdown-order workaround is authorized. The defect
reproduces with `ros2 launch` removed entirely — the bare `move_group` ELF, with
no `ros2_control`, no spawners and no other node, crashes identically.

**Test environments must export `DEBUGINFOD_URLS=""`.** This is not a cosmetic
preference. With it unset, teardown is bounded and deterministic (~1.5 s); with
it populated, a crashed `move_group` blocks on the network for an unbounded
time and the true failure signal is destroyed. This is an environment
requirement for the test harness, not a change to `real_hardware.launch.py`.

Ubuntu populates `DEBUGINFOD_URLS` from `/etc/profile.d`, so it is the default
in an ordinary shell and the requirement cannot be left to the caller. It is
enforced in two places: `src/arm_tests/CMakeLists.txt` pins it on the
`test_m3_launch_smoke` CTest entry for the `colcon test` path, and the test
module itself refuses to run if it is populated, which covers direct
`launch_test` invocations.

## Exact MoveIt exception

Exception identifier: `MOVEIT_JAZZY_2_12_4_TEARDOWN_SIGSEGV`  
(supersedes `MOVEIT_JAZZY_2_12_4_SHUTDOWN_LIFETIME_ORDER`)

The exception is permitted only when every identity check matches. **The
identity is per-architecture**: SHA-256 and ELF build ID are properties of a
compiled binary, so the amd64 row below can never authorise an arm64 build.

| Identity | Required value |
| --- | --- |
| ROS distribution | `jazzy` |
| OS | Ubuntu `24.04` |
| Debian package | `ros-jazzy-moveit-ros-move-group` |

| Architecture | Debian version | Executable SHA-256 | ELF build ID |
| --- | --- | --- | --- |
| `amd64` | `2.12.4-1noble.20260903.094420` | `45fdd6ee094aa9c8ad2df411548bc7d2bf5180375d234043ed9ef0aa6d4c6829` | `e8b3bf99f898525e381412e6e0c12dbb5715216b` |
| `arm64` (Pi 5) | **not yet measured — fails closed** | — | — |

An architecture with no row fails closed: the exception is refused and
`move_group` must exit `0` like everything else. Adding a row requires
measuring the defect on that architecture with
`scripts/repro_move_group_shutdown.sh` and recording the values here. The
machine-readable copy lives in `MOVEIT_EXCEPTION_IDENTITIES` in
`src/arm_tests/integration/m3_process_assertions.py`; the two must agree.

Missing metadata or any mismatch disables the exception. A package upgrade,
downgrade, source build, or changed executable therefore returns the contract to
strict exit `0` until a new evidence review updates this document.

The 2026-10-02 identity was remeasured after the Jazzy Debian package revision
changed. Three consecutive isolated runs ended in approximately 1.22 s with
`rc=139`, `Deleting MoveItCpp`, and the same
`rclcpp::CallbackGroup::~CallbackGroup()` terminal frame. The full M3 launch
also reproduced `move_group` exit `-11` after successful liveness and controller
activation; the other long-running processes shut down normally.

For the exact artifact above, `move_group` exit `-11` (SIGSEGV, shell `rc=139`)
is accepted only if all of these runtime predicates hold:

1. `DEBUGINFOD_URLS` is empty in the process environment.
2. `move_group` passed the same bounded liveness deadline as
   `ros2_control_node`, `robot_state_publisher`, and `robot_arm_pick_place_node`.
3. All three controllers were proven `active` through
   `/controller_manager/list_controllers` during the acceptance window.
4. Shutdown began only after the test requested it.
5. Output contains `Deleting MoveItCpp`, and the crash frame reported by
   `backward_ros` is `rclcpp::CallbackGroup::~CallbackGroup()`.
6. The child exit is exactly `-11`. A `-15` now means the debuginfod
   requirement was violated or the failure changed — both are failures.
7. The other three long-lived processes and all three spawners exit `0`.
8. Controller and mock hardware deactivate/shutdown success appears before the
   `ros2_control_node` process exits.
9. The result reports the exception identifier and installed version visibly in
   test/JUnit output. It is not a skip and does not turn an unknown failure green.

Any different signal, exit code, ordering, early death, missing process, missing
controller proof, or extra nonzero exit is a failure.

## Evidence

### Isolated reproduction

`scripts/repro_move_group_shutdown.sh` runs the `move_group` ELF directly. No
launch file, no `ros2_control`, no spawners, no other node participate:

```bash
scripts/repro_move_group_shutdown.sh              # canonical: DEBUGINFOD_URLS=""
scripts/repro_move_group_shutdown.sh --debuginfod # A/B: reproduces the fake "hang"
```

Canonical mode, three consecutive trials: `rc=139` after 11.8 s, 1.7 s and
1.6 s, each ending in
`Segmentation fault (Address not mapped to object)` with frame `#0` at
`rclcpp::CallbackGroup::~CallbackGroup()`.

### Native backtrace

Captured under gdb as the parent process (`ptrace_scope=1` forbids attaching).
The main thread was sampled at t+5 s, t+10 s and t+30 s after SIGINT across
three independent runs and was in the identical frame every time:

```
#0  rclcpp::CallbackGroup::~CallbackGroup()          librclcpp.so
#2  rclcpp::node_interfaces::NodeBase::~NodeBase()   librclcpp.so
#5  rclcpp::Node::~Node()                            librclcpp.so
#7  trajectory_execution_manager::TrajectoryExecutionManager::~TrajectoryExecutionManager()
#9  moveit_cpp::MoveItCpp::~MoveItCpp()              libmoveit_cpp.so.2.12.4
#11 main
```

So the fault is in the `MoveItCpp` destructor chain, while
`TrajectoryExecutionManager` tears down its internal `rclcpp::Node`. This is
independent of robot geometry, SRDF and kinematics: the crashing object is a
plain node owned by MoveIt.

Supporting measurements taken while the process appeared hung (debuginfod on):

- Main-thread CPU jiffies were unchanged over 4 s (`59 12` → `59 12`), so it was
  genuinely blocked, not spinning.
- `/proc/<tid>/wchan` for the main thread read `poll_schedule_timeout`, not a
  futex — i.e. a socket wait, not a mutex deadlock.
- `strace` showed an endless `poll([{fd=64}], 1, 1000)` retry loop, and
  `fd=64` resolved to an ESTABLISHED TCP socket to `91.189.92.252:443`
  (`debuginfod.ubuntu.com`), alongside an open
  debuginfod cache entry for the measured binary's build ID.

### Controlled A/B on the full M3 launch

Same script, same launch, one variable changed:

| `DEBUGINFOD_URLS` | Teardown | `ros2 launch` rc | `move_group` |
| --- | --- | --- | --- |
| `""` | 1.67 s | `0` | exit `-11` (SIGSEGV) |
| `https://debuginfod.ubuntu.com` | >90 s, never completed | — | never died; launch escalates to SIGTERM → `-15` |

In the clean run all six other processes reported `process has finished
cleanly`: `robot_state_publisher`, `ros2_control_node`, `robot_arm_pick_place_node`
and the three spawners. `move_group` is the only failing process.

### Upstream status

- The installed Jazzy package revision measured on 2026-10-02 is
  `2.12.4-1noble.20260903.094420`; it reproduces the same teardown defect as the
  previously recorded revision. A package revision must not inherit the
  exception without repeating the identity and reproduction gates above.
- Upstream [moveit2 issue #3721](https://github.com/moveit/moveit2/issues/3721),
  "MoveItPy segmentation fault on teardown during object destruction (ROS 2
  Jazzy, MoveIt 2.12.4)", is **open** and reports the same version, the same
  `Deleting MoveItCpp` phase and the same segfault-on-teardown signature on a
  completely unrelated robot (`mrs15_3120`). Two independent robot
  configurations crashing identically at the same teardown phase of the same
  binary is the attribution argument for an upstream defect.
- [Jazzy PR #3486](https://github.com/moveit/moveit2/pull/3486) is already an
  ancestor of tag `2.12.4`, so this must not be labelled as that
  already-backported `Rate::sleep()` defect.

### Honest limits of this evidence

- The distribution binaries are stripped and no ROS debuginfo is installed, so
  the backtrace resolves to exported symbols only. The faulting *line* inside
  `CallbackGroup::~CallbackGroup()` is not proven; the faulting *destructor
  chain* is.
- The crash was not reproduced against a stock upstream MoveIt config
  (`moveit_resources` is not installed here, and installing it would change the
  measured environment). Attribution to upstream rests on the isolated bare-ELF
  reproduction plus upstream issue #3721, not on a local stock-config run.
- All measurements are from this amd64 workstation. The Pi 5 is arm64 and has
  not been measured, so it currently has no identity row and the exception is
  refused there by design. Expect the first Pi 5 M3 run to report
  `move_group` exit `-11` as a *failure* until the defect is measured there and
  an `arm64` row is added. That is the intended fail-closed behaviour, not a
  regression.

## `pal_statistics` message policy

The message

```text
Exception in publisher thread: context cannot be slept with because it's invalid!. Aborting!
```

is not attributed to `realtime_tools` issue
[#480](https://github.com/ros-controls/realtime_tools/issues/480). That issue was
closed as stale/not-planned without a reproducing stack trace, and a maintainer
explicitly stated that the referenced realtime clock is not used on Jazzy.

The exact text originates in `pal_statistics` 2.7.0: its publisher thread uses
`rclcpp::WallRate::sleep()`, catches `std::exception`, logs the message, and then
leaves its outer loop after the ROS context becomes invalid. The source is
[pal_statistics.cpp at tag 2.7.0](https://github.com/pal-robotics/pal_statistics/blob/2.7.0/pal_statistics/src/pal_statistics.cpp#L342-L367).

This log is a visible warning, not a process-exit exception, only for this exact
pair:

- `ros-jazzy-pal-statistics=2.7.0-1noble.20260615.144746`
- `ros-jazzy-controller-manager=4.45.2-1noble.20260615.164916`

It is tolerated only after successful controller and hardware shutdown and only
when `ros2_control_node` exits `0`. The 2026-08-25 clean A/B run confirms it:
the message appears, and `ros2_control_node` still reports `process has finished
cleanly`. A different version, earlier occurrence, different message, failed
controller/hardware shutdown, or nonzero process exit is a failure and requires
review. `pal_statistics` 2.8.0 retains the same loop, so an upgrade does not
inherit the exception automatically; the version gate must still fail closed and
be reviewed.

## Removal condition

Remove the MoveIt exception when an installed upstream package or an explicitly
reviewed source patch exits cleanly in
`scripts/repro_move_group_shutdown.sh` and in the full M3 test. Required
evidence is three clean isolated shutdowns plus the full M3 liveness,
active-controller, seven-process exit, and negative-fixture suite.
