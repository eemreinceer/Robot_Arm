"""Shared process assertions and controller verifications for M3 launch smoke tests."""

import hashlib
import os
import subprocess
import sys
import time

REQUIRED_LONG_RUNNING_PROCESSES = (
    "ros2_control_node",
    "robot_state_publisher",
    "move_group",
    "robot_arm_pick_place_node",
)

EXPECTED_ACTIVE_CONTROLLERS = (
    "joint_state_broadcaster",
    "robot_arm_controller",
    "robot_arm_gripper_controller",
)


# Identity of the exact MoveIt build that may claim the teardown-segfault
# exception, keyed by Debian architecture. SHA-256 and ELF build ID are
# per-architecture: the amd64 row below CANNOT authorise an arm64 Pi 5 build.
# Architectures absent from this table fail closed by design -- add a row only
# after measuring the defect on that architecture and updating
# docs/m3_shutdown_policy.md.
MOVEIT_EXCEPTION_IDENTITIES = {
    "amd64": {
        "debian_version": "2.12.4-1noble.20260903.094420",
        "sha256": "45fdd6ee094aa9c8ad2df411548bc7d2bf5180375d234043ed9ef0aa6d4c6829",
        "build_id": "e8b3bf99f898525e381412e6e0c12dbb5715216b",
    },
}

EXCEPTION_ID = "MOVEIT_JAZZY_2_12_4_TEARDOWN_SIGSEGV"


def _dpkg_architecture():
    try:
        res = subprocess.run(
            ["dpkg", "--print-architecture"], capture_output=True, text=True, check=False
        )
        if res.returncode == 0:
            return res.stdout.strip()
    except OSError:
        pass
    return ""


def _move_group_executable():
    """Locate the running move_group binary without assuming /opt/ros/jazzy."""
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix("moveit_ros_move_group")
        candidate = os.path.join(prefix, "lib", "moveit_ros_move_group", "move_group")
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    fallback = "/opt/ros/jazzy/lib/moveit_ros_move_group/move_group"
    return fallback if os.path.isfile(fallback) else ""


def _sha256(path):
    """hashlib, not shasum: shasum is a perl script and is absent on minimal images."""
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _elf_build_id(path):
    """Read .note.gnu.build-id straight out of the ELF, so binutils is not required.

    Returns "" when the note cannot be found; every caller treats that as a
    mismatch, so an unreadable binary can never grant the exception.
    """
    try:
        with open(path, "rb") as handle:
            data = handle.read()
        if data[:4] != b"\x7fELF" or data[4] != 2:  # 64-bit ELF only
            return ""
        little = data[5] == 1
        order = "little" if little else "big"

        def num(offset, size):
            return int.from_bytes(data[offset:offset + size], order)

        e_shoff = num(0x28, 8)
        e_shentsize = num(0x3A, 2)
        e_shnum = num(0x3C, 2)
        e_shstrndx = num(0x3E, 2)
        if not e_shoff or not e_shnum:
            return ""

        strtab_hdr = e_shoff + e_shstrndx * e_shentsize
        strtab_off = num(strtab_hdr + 0x18, 8)

        for i in range(e_shnum):
            hdr = e_shoff + i * e_shentsize
            name_off = strtab_off + num(hdr, 4)
            name_end = data.index(b"\x00", name_off)
            if data[name_off:name_end] != b".note.gnu.build-id":
                continue
            sec_off = num(hdr + 0x18, 8)
            sec_size = num(hdr + 0x20, 8)
            note = data[sec_off:sec_off + sec_size]
            n_namesz = int.from_bytes(note[0:4], order)
            n_descsz = int.from_bytes(note[4:8], order)
            desc_start = 12 + ((n_namesz + 3) // 4) * 4
            return note[desc_start:desc_start + n_descsz].hex()
    except Exception:
        return ""
    return ""


def get_moveit_teardown_exception_status():
    """Decide whether this exact MoveIt build may claim the teardown-segfault exception.

    Fails closed on every uncertainty: unknown architecture, missing metadata,
    unreadable binary or any mismatch. See docs/m3_shutdown_policy.md.
    """
    if os.environ.get("ROS_DISTRO") != "jazzy":
        return False, f"ROS_DISTRO is not jazzy: {os.environ.get('ROS_DISTRO')!r}"

    arch = _dpkg_architecture()
    expected = MOVEIT_EXCEPTION_IDENTITIES.get(arch)
    if expected is None:
        return False, (
            f"no measured exception identity for architecture {arch!r}; "
            f"known: {sorted(MOVEIT_EXCEPTION_IDENTITIES)}. Measure the defect on "
            "this architecture and update docs/m3_shutdown_policy.md before "
            "allowing the exception."
        )

    res = subprocess.run(
        ["dpkg-query", "-W", "-f=${Version}", "ros-jazzy-moveit-ros-move-group"],
        capture_output=True,
        text=True,
        check=False,
    )
    installed = res.stdout.strip() if res.returncode == 0 else ""
    if installed != expected["debian_version"]:
        return False, f"Debian version mismatch on {arch}: {installed!r}"

    exe_path = _move_group_executable()
    if not exe_path:
        return False, "move_group executable not found"

    sha = _sha256(exe_path)
    if sha != expected["sha256"]:
        return False, f"SHA256 mismatch on {arch}: {sha}"

    build_id = _elf_build_id(exe_path)
    if build_id != expected["build_id"]:
        return False, f"Build ID mismatch on {arch}: {build_id!r}"

    return True, f"{EXCEPTION_ID} ({arch})"


def assert_required_processes_live(
    proc_info,
    required_names=REQUIRED_LONG_RUNNING_PROCESSES,
    window_seconds=3.0,
    startup_timeout=15.0,
):
    """Require every named process to start and survive one shared deadline."""
    from launch.events.process import ProcessExited

    startup_deadline = time.monotonic() + startup_timeout
    for name in required_names:
        try:
            remaining = max(0.1, startup_deadline - time.monotonic())
            proc_info.assertWaitForStartup(process=name, timeout=remaining)
        except AssertionError as exc:
            raise AssertionError(f"mandatory process missing: {name}") from exc

    alive_deadline = time.monotonic() + window_seconds
    while time.monotonic() < alive_deadline:
        for name in required_names:
            if isinstance(proc_info[name], ProcessExited):
                raise AssertionError(
                    f"mandatory process exited inside alive window: {name}"
                )
        time.sleep(min(0.05, max(0.0, alive_deadline - time.monotonic())))


def verify_active_controllers(
    expected_controllers=EXPECTED_ACTIVE_CONTROLLERS,
    timeout_seconds=25.0,
):
    """Call /controller_manager/list_controllers and assert all expected controllers are active."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from controller_manager_msgs.srv import ListControllers

    context = rclpy.context.Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        f"m3_ctrl_verify_{os.getpid()}_{int(time.time() * 1000) % 100000}",
        context=context,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    client = node.create_client(
        ListControllers,
        "/controller_manager/list_controllers",
    )

    deadline = time.monotonic() + timeout_seconds
    try:
        service_ready = False
        while time.monotonic() < deadline:
            if client.service_is_ready():
                service_ready = True
                break
            executor.spin_once(timeout_sec=0.2)

        if not service_ready:
            raise AssertionError(
                f"Service /controller_manager/list_controllers not available after {timeout_seconds}s"
            )

        active_set = set()
        while time.monotonic() < deadline:
            future = client.call_async(ListControllers.Request())
            executor.spin_until_future_complete(future, timeout_sec=1.5)
            if future.done() and not future.cancelled() and future.exception() is None:
                resp = future.result()
                if resp is not None:
                    active_set = {
                        c.name for c in resp.controller if c.state == "active"
                    }
                    if all(c in active_set for c in expected_controllers):
                        return {c.name: c.state for c in resp.controller}
            executor.spin_once(timeout_sec=0.1)

        missing = [c for c in expected_controllers if c not in active_set]
        raise AssertionError(
            f"Controllers did not reach 'active' state within {timeout_seconds}s: "
            f"missing {missing}, found active {sorted(active_set)}"
        )
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def assert_processes_shutdown_cleanly(
    proc_info,
    required_processes=REQUIRED_LONG_RUNNING_PROCESSES,
):
    """Assert exit codes for all processes with fail-closed upstream MoveIt exception."""
    is_allowed, auth_reason = get_moveit_teardown_exception_status()
    failures = []
    upstream_exceptions = []

    raw_processes = list(proc_info.processes())
    if not raw_processes:
        raise AssertionError("No processes recorded in proc_info during shutdown")

    found_required = set()

    for item in raw_processes:
        try:
            info = proc_info[item]
        except Exception:
            info = item

        name = getattr(info, "process_name", str(info))
        exit_code = getattr(info, "returncode", None)
        pid = getattr(info, "pid", "unknown")

        for req_proc in required_processes:
            if req_proc in name:
                found_required.add(req_proc)

        is_move_group = "move_group" in name

        if exit_code == 0:
            continue
        elif is_move_group and exit_code == -11:
            if is_allowed:
                upstream_exceptions.append(
                    f"move_group (pid {pid}) exited with code {exit_code} "
                    f"due to upstream teardown segfault (Issue #14 allowlisted: {auth_reason})"
                )
            else:
                failures.append(
                    f"Process {name} (pid {pid}) exited with unauthorized code {exit_code} "
                    f"(MoveIt exception gate failed: {auth_reason})"
                )
        else:
            failures.append(
                f"Process {name} (pid {pid}) exited with non-zero code {exit_code}"
            )

    missing_processes = set(required_processes) - found_required
    if missing_processes:
        failures.append(
            "Missing required processes in teardown verification: "
            f"{sorted(missing_processes)}"
        )

    for exc_msg in upstream_exceptions:
        print(f"\n[UPSTREAM ALLOWLIST EXCEPTION] {exc_msg}\n", file=sys.stderr)

    if failures:
        raise AssertionError("Shutdown process exit code assertions failed:\n" + "\n".join(failures))
