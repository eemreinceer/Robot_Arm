# Repository Artifact Inventory

This inventory separates source inputs, unique measurements and reproducible
outputs. It reflects the tracked tree as of 2026-10-02.

## Current hygiene state

- `git ls-files -ci --exclude-standard` returns **0** tracked-but-ignored files.
- Root-level `export.log`, duplicate `meshes/`, `yolov8n.pt` and `yolo26n.pt`
  artifacts are not present in the tracked tree.
- Colcon output (`build/`, `install/`, `log/`), Python caches, editor state and
  local credentials are ignored.
- Recorded measurements are allowlisted narrowly under `runs/`; raw capture
  streams and reproducible bulk outputs remain untracked.

## Runtime model inventory

| Path | Size (bytes) | SHA-256 | Status |
| --- | ---: | --- | --- |
| `src/arm_ml/models/ik_net.pt` | 2,149,349 | `bc06ecabd0c92af1c5dbafdd9a5b61ebf4635a5aeda18add622ae057fb718f4d` | Legacy experiment; package is outside the active build |
| `src/arm_ml/models/ik_net_scripted.pt` | 2,170,153 | `2457e2ab1e6e9b5208a6778b0e8516fb2a0e4675daf73b061727fec615ba3c1c` | Legacy TorchScript experiment; package is outside the active build |
| `src/arm_perception/models/yolo_arm.pt` | 6,240,810 | `c4514564dad70f4538609c3430b7e65ebd3ebee22dd2e0e8fde7e3dfd2ade87b` | Active perception input; redistribution terms must be checked before any open-source release |

The repository-level license does not claim ownership of third-party model
formats, base weights or trademarks. A model must have its upstream source,
license, training command, dataset version, metrics, producing commit and
target runtime recorded before it can be redistributed as an independently
licensed release artifact.

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

# Review tracked model identity and size.
sha256sum src/arm_ml/models/*.pt src/arm_perception/models/*.pt

# Confirm repository links and baseline policy.
./scripts/verify_workspace.sh --quick
```

History cleanup and working-tree cleanup are different operations. Removed
artifacts may remain in earlier Git objects until an explicitly approved
history rewrite or a clean public export is performed.
