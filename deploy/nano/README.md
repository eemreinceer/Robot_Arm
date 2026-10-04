# Nano deployment surface

The container definitions for the Jetson Nano. This directory exists because
the deployment surface was, until 2026-08-05, the one layer with no version
control and no evidence.

## The problem this closes

Two defects that were already fixed in this repo came back on 2026-08-05, and
both came back the same way: the fix landed on `main`, and the container that
actually runs kept its original hand-typed command across every reboot.

| Defect | Fixed in repo | Still broken on the Nano |
| --- | --- | --- |
| CycloneDDS not pinned to `eth0` | 2026-07-28 | container had no `CYCLONEDDS_URI` at all |
| `camera_info_file` not passed | 2026-07-27 | container ran `ros2 run`, not the launch file |

The second one cost ten days of a silently uncalibrated ROS graph. Nobody was
careless; there was simply nothing that compared the running container to the
repo, and nothing that failed when they diverged.

## What is here

| File | Purpose |
| --- | --- |
| `robot_arm_camera.sh` | Canonical definition of the camera container. Runs on the PC, drives the Nano over SSH, like `start_robot.sh`. |
| `cyclonedds_eth0.xml` | DDS interface pinning. |
| `mjpeg_bridge.py` | Serves `/camera/image_raw` as MJPEG on `:8080`, and is PID 1 of the container. |

`--recreate` ships both data files to the Nano, so these copies are the source
of truth. If the Nano copy differs, it is preserved as
`<name>.pre_deploy_<timestamp>` before being replaced — an unversioned edit
nobody captured is what this directory exists to end, but it should not be
destroyed on the way out either.

`mjpeg_bridge.py` was taken in **byte-identical** to the copy that had been
running (sha256 `5b4d3521…aeb858`), deliberately unmodified. Changing it in the
same step would mean the repo copy is no longer the copy that was proven to
work. Two things worth fixing later, noted rather than done: the stream handler
spins on `continue` while waiting for the first frame, and `import time` sits
inside that loop.

It lives here rather than in `src/arm_perception/` for two reasons: that package
is part of the main workspace, and installing it as a ROS entry point would require rebuilding the
Nano overlay — a real risk for a file that already works, while the arm is
blocked on other things.

## Use

```bash
deploy/nano/robot_arm_camera.sh --status      # read-only: what is actually running
deploy/nano/robot_arm_camera.sh --recreate    # rebuild from this definition
deploy/nano/robot_arm_camera.sh --verify      # acceptance gate
```

`--recreate` renames the existing container instead of deleting it, and
disables its restart policy. Two camera containers with `unless-stopped` would
both start at the next boot and fight over the Argus socket and port 8080.

Backups therefore accumulate as `robot_arm_camera_backup_<timestamp>`. Nothing prunes
them, on purpose — deleting a container that might be the last working one
should be a deliberate act, not a side effect of a deploy script.

The preflight is fail-closed and runs before anything is touched. Verified by
negative test on 2026-08-05: with `ROBOT_ARM_DDS_IFACE=nonexistent0` it reports the
dead interface, exits 1, and leaves the running container alone.

## Why the gate runs from the PC

`--verify` is `scripts/check_alive.sh --camera`, run on the PC on purpose. The
2026-08-05 DDS failure was completely invisible on the Nano: the container was
`Up`, the node was publishing at 30 Hz, `:8080` served frames, and
`ros2 topic info` showed a publisher. Every local signal was green. Only the
other machine could see that nothing was arriving.

Two things the gate knows that a casual check does not:

* A topic stays visible with **zero publishers** if something merely subscribes
  to it. `check_alive.sh` reports the publisher count separately from the rate
  so this case is named instead of looking like a slow topic.
* `ros2 node list` comes back **empty** across the PC's Jazzy and the Nano's
  Humble even when image data flows fine — measured. The rates are the truth;
  the listing is not.

## Health is not container state

The container stays `Up` after the camera dies. `mjpeg_bridge.py` is PID 1 and
the launch file runs beside it, so if the launch retry budget (2 attempts / 5 s)
is exhausted, the launch shuts down and the container carries on serving an
empty stream. `restart: unless-stopped` will not catch this.

The health gate is `--verify`, or `curl :8088/api/v1/health` for the web
console. Never `docker ps`.

## Still unversioned

The Nano workspace overlay `${ROBOT_ARM_WS_HOST:-/opt/robot_arm/robot_ws}/install/` is still built on the
Nano. The preflight refuses to build a container around a missing overlay, but
it cannot prove the overlay matches this repo — the stale-`install/` trap that
has bitten this project before is still open.

`robot_arm_hw` (the robot control container, `--device /dev/ttyTHS1`) is also not
defined here yet. It has the same exposure. It was left alone deliberately:
that container is bound up with the servo rail and the fail-closed bring-up
sequence in `start_robot.sh`, and it should not be rewritten while the arm is
blocked on the q=0 repeatability issue.
