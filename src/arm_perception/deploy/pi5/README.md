# Pi 5 camera and dashboard services

These user services keep the read-only dashboard and the IMX219 ROS publisher
available while the Pi 5 is running. They do not start ros2_control, open the
ESP32 serial port, enable PWM, or issue motion commands.

The camera service explicitly selects the preserved `/usr/local` GStreamer
plugin. A non-interactive service must not rely on `.bashrc`; without the
explicit `GST_PLUGIN_PATH`, GStreamer can select apt's libcamera 0.2.0 plugin
instead of the installed 0.7.x build.

The camera unit uses `Restart=always` and runs the installed camera executable
directly as its tracked main process. A `ros2 launch` parent is intentionally
not used here: if only that parent is terminated, an orphan camera child can
remain and a restart can create two publishers. `KillMode=control-group`
provides full cleanup for controlled stop/restart operations.

## Install or update

Create the runtime root and optional environment file, then copy both unit
files to `~/.config/systemd/user/`:

```bash
sudo install -d -o "$(id -un)" -g "$(id -gn)" /opt/robot_arm
sudo install -d /etc/robot_arm
printf '%s\n' 'ROBOT_ARM_RUNTIME_ROOT=/opt/robot_arm' 'ROBOT_ARM_BIND_ADDR=127.0.0.1' |
  sudo tee /etc/robot_arm/robot_arm.env >/dev/null
```

Enable the user services:

```bash
systemctl --user daemon-reload
systemctl --user enable --now robot-arm-camera.service robot-arm-dashboard.service
loginctl enable-linger "$(id -un)"
```

Linger keeps the user service manager alive without an interactive SSH or
desktop session. Verify it with `loginctl show-user "$(id -un)" -p Linger`;
the expected value is `Linger=yes`.

The units consume the release selected by
`${ROBOT_ARM_RUNTIME_ROOT:-/opt/robot_arm}/current`. Update that symlink atomically
before restarting the services when deploying a new release.

## Verify

```bash
systemctl --user is-active robot-arm-camera.service robot-arm-dashboard.service
loginctl show-user "$(id -un)" -p Linger
ros2 topic info /camera/image_raw --verbose
ros2 topic info /camera/camera_info --verbose
timeout 12 ros2 topic hz /camera/image_raw
curl -fsS http://127.0.0.1:8088/api/v1/health
```

Acceptance requires one camera publisher, a measured non-zero image rate,
matching image/CameraInfo dimensions and a dashboard camera state of `online`.
An HTTP 200 response by itself is not camera evidence.

## Roll back

```bash
systemctl --user disable --now robot-arm-camera.service
systemctl --user restart robot-arm-dashboard.service
```

This leaves the dashboard available but returns camera startup to manual
operation. To roll back an entire dashboard release, point `current` to the
previous release recorded in `${ROBOT_ARM_RUNTIME_ROOT:-/opt/robot_arm}/PREVIOUS_RELEASE`
and restart both services.
