# Phase 3: IK Solvers Comparison Report (Analytic DLS vs Deep Learning vs MoveIt)

Generated on: 2026-05-30 19:54:06
Reachable samples (FK of random valid joints): 500
Accuracy gate: position error <= 5.0 mm (Link_6 FK reconstruction)

## Methodology
Targets are produced by sampling random joints within the canonical self-collision
JOINT_LIMITS and running `/fk_solve` — every target is therefore reachable, with a
real orientation (not a fixed identity box). Accuracy is the Link_6 FK-reconstruction
position error of each solver's returned joints: `/ik_solve` and `/dl_ik_solve` report
`position_error` directly; MoveIt's solution is run back through `/fk_solve`.

## Results

| Solver | Accurate (≤5mm) | Returned | Err Mean (mm) | Err Median (mm) | Err p95 (mm) | Err Max (mm) | Lat Avg (ms) | Lat Median (ms) | Lat p95 (ms) |
|---|---|---|---|---|---|---|---|---|---|
| **MoveIt2 (pick_ik)** | 100.00% | 100.00% | 0.001 | 0.000 | 0.005 | 0.009 | 8.6494 | 2.2932 | 22.0527 |
| **Custom DLS (`arm_kinematics`)** | 27.60% | 27.60% | 0.024 | 0.014 | 0.081 | 0.098 | 38.4722 | 49.8548 | 54.3585 |
| **Deep Learning IK (`arm_ml`)** | 2.20% | 100.00% | 17.555 | 15.190 | 32.960 | 108.992 | 1.9073 | 1.8366 | 2.6653 |

## Notes
- **Accurate** = fraction whose FK-reconstruction error is within the 5 mm gate.
  **Returned** = fraction where the solver reported a solution at all. For the DL net
  these differ sharply because it always returns a vector regardless of accuracy.
- DL IK is intended as a fast seed generator; pair it with analytic DLS / MoveIt
  refinement for precise picks (mean error well above the 5 mm gate).
- Frames are Link_6 throughout; grasp offset is applied in the Phase 4 pick layer.
