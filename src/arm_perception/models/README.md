# Perception model provisioning

The public repository does not distribute trained detector weights. Model
files can contain third-party base-model obligations and dataset-derived
information, so they remain local until provenance and redistribution rights
are documented.

The default runtime path is:

```text
src/arm_perception/models/yolo_arm.pt
```

Provision that file locally before starting a perception or physical
acceptance workflow. A publishable model artifact must record:

- upstream model and license;
- dataset identity and permission to use it;
- training and export commands;
- dependency and target-runtime versions;
- producing commit and SHA-256 digest;
- validation metrics and their evidence class.

`arm_perception.train_yolo` can produce the expected filename. The file is
ignored by Git; a missing model must fail at the perception-node boundary
rather than silently substituting unrelated weights.
