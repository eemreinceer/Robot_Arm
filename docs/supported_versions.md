# Supported environments and versions

This document records the development and target-environment contract. It is
not a physical-acceptance or release statement.

## Development workstation

| Component | Supported or measured value | Status |
| --- | --- | --- |
| Operating system | Ubuntu 24.04 LTS | Active development host |
| ROS 2 | Jazzy | Canonical host distribution |
| Python | 3.12 | Static checks and ROS Python packages |
| PlatformIO Core | 6.1.19 | Pinned in native firmware CI |
| ShellCheck | 0.9.0 | Measured local baseline |
| Node.js | 22 | Web-console CI baseline |

## Raspberry Pi 5 target — active

Measured on the device on 2026-08-26 (`ssh pi5`).

| Component | Measured value | Note |
| --- | --- | --- |
| Operating system | Ubuntu 24.04.4 LTS | Same base as the host |
| Kernel / architecture | 6.8.0-1061-raspi / `aarch64` | Raspberry Pi kernel |
| ROS 2 | **Jazzy** | Same distribution as the host; no separate container required |
| Python | 3.12.3 | Same major/minor as the host |
| OpenCV | 4.6.0 | Used by the camera node's GStreamer path |
| GStreamer | 1.24.2 | IMX219 through `libcamerasrc` |
| libcamera from apt | 0.2.0 | **Not used**; incompatible with the camera node |
| locally built libcamera | **0.7.2+rpt20260817** | Selected through the unit's `GST_PLUGIN_PATH` |
| colcon | installed | Device-side builds are supported |

The apt libcamera 0.2.0 and locally built 0.7.2 coexist. The
`robot-arm-camera.service` unit deliberately selects the local build. Removing
that `GST_PLUGIN_PATH` can make the camera silently load the wrong plugin.

The deployment contract and version-controlled systemd unit copies live in
[`deploy/pi5/README.md`](../deploy/pi5/README.md).

## Jetson Nano target — retired

> Jetson Nano was retired as the active target in 2026-08. The following values
> are retained as historical engineering evidence, not as a baseline for new
> work.

| Component | Measured value | Note |
| --- | --- | --- |
| JetPack/L4T family | JetPack 4.x-compatible surface | Preserve only for historical reproduction |
| ROS 2 | Humble in a container | Do not source a Jazzy install tree here |
| Host Python | 3.6.9 | Scripts deployed directly to Nano preserve this grammar |
| CUDA | 10.2.300 | Measured in 2026-07 |
| TensorRT | 8.2.1 / Python 8.2.1.8 | Target-device measurement |
| Ultralytics export host | 8.4.75 | Historical PT to ONNX export environment |
| ONNX | 1.22.0, opset 12 | Historical Nano TensorRT path |

See [`reports/nano_e2b_tensorrt.md`](../reports/nano_e2b_tensorrt.md) and
[`deploy/nano/README.md`](../deploy/nano/README.md) for the historical evidence.

## Firmware targets

| Environment | Target | Framework | Verification |
| --- | --- | --- | --- |
| `firmware/stm32_servo_ctrl` | `bluepill_f103c8` | Arduino / PlatformIO | Device build plus native protocol tests |
| `firmware/esp32_servo_ctrl` | `esp32dev` | Arduino / PlatformIO | Device build plus native motion/protocol tests |

PlatformIO Core is pinned in CI, but the platform packages in
`platformio.ini` are not yet pinned. This is a documented reproducibility gap
that must be closed before a versioned firmware release.

## Models and releases

- Trained model weights are not distributed. Provisioning and provenance rules
  are documented in [`repository_artifact_inventory.md`](repository_artifact_inventory.md).
- The project has no semantic release yet, so no inferred `VERSION` value is
  published.
- The repository is available for portfolio evaluation under the custom
  evaluation-only terms in [`LICENSE`](../LICENSE).

When updating this matrix, record the measurement date, target device and tool
version together.
