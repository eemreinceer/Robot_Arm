# Repository Artifact Inventory

This inventory separates source inputs, unique measurements and reproducible
outputs. It reflects the tracked tree as of 2026-10-04.

## Current hygiene state

- `git ls-files -ci --exclude-standard` returns **0** tracked-but-ignored files.
- Root-level `export.log`, duplicate `meshes/`, `yolov8n.pt` and `yolo26n.pt`
  artifacts are not present in the tracked tree.
- Colcon output (`build/`, `install/`, `log/`), Python caches, editor state and
  local credentials are ignored.
- Recorded measurements are allowlisted narrowly under `runs/`; raw capture
  streams and reproducible bulk outputs remain untracked.

## Runtime model policy

No trained model weights are distributed in the public repository. Expected
local artifacts are documented without granting or implying redistribution
rights:

| Local path | Purpose | Public provisioning contract |
| --- | --- | --- |
| `src/arm_ml/models/ik_net.pt` | Legacy IK experiment checkpoint | [`src/arm_ml/models/README.md`](../src/arm_ml/models/README.md) |
| `src/arm_ml/models/ik_net_scripted.pt` | Legacy TorchScript IK experiment | [`src/arm_ml/models/README.md`](../src/arm_ml/models/README.md) |
| `src/arm_perception/models/yolo_arm.pt` | Active detector input | [`src/arm_perception/models/README.md`](../src/arm_perception/models/README.md) |

These files are ignored by Git. A model may be published separately only after
its upstream source, license, training command, dataset permission, metrics,
producing commit, SHA-256 digest and target runtime have been recorded.

## Measurement and report policy

| Group | Repository policy |
| --- | --- |
| `runs/charuco_*.yaml`, camera calibration YAML | Unique calibration inputs; tracked with date and target context |
| `runs/joint_limits_*.yaml`, `runs/zero_offset_*.yaml` | Physical measurements; tracked because reproduction requires the robot |
| `runs/hand_eye/*.json` | Calibration evidence; tracked only through explicit allowlist entries |
| `reports/faz4_dataset_bug/` | Minimal visual evidence supporting the associated diagnosis |
| `reports/final_rapor/` | Final report source and PDF; intermediate rendered pages are ignored |
| `runs/detect/` bulk output | Reproducible inference output; excluded except for its explanatory README |

Every numerical claim intended for the dashboard must point to a tracked
artifact, a tracked analysis script plus input, or a report containing the
reproduction command. A deleted chat, coordination log or local-only path is
not valid provenance.

## Verification commands

```bash
# Must print nothing.
git ls-files -ci --exclude-standard

# Confirm that model weights remain untracked.
git ls-files '*.pt' '*.pth' '*.onnx'

# Confirm repository links and baseline policy.
./scripts/verify_workspace.sh --quick
```

History cleanup and working-tree cleanup are different operations. Removed
artifacts may remain in earlier Git objects until an explicitly approved
history rewrite or a clean public export is performed.
