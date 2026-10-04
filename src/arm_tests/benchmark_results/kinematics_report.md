# Kinematics Performance Benchmark Report

Generated on: 2026-05-30 01:08:54  
Number of samples: 1000  

## Latency and Success Rate Analysis

| Solver / Service | Success Rate | Avg Latency (ms) | Median Latency (ms) | p95 Latency (ms) | p99 Latency (ms) | Min Latency (ms) | Max Latency (ms) |
|---|---|---|---|---|---|---|---|
| **MoveIt2 FK** (`/compute_fk`) | 100.00% | 1.2560 | 0.9815 | 3.0150 | 5.5900 | 0.4853 | 7.9977 |
| **Custom FK** (`/fk_solve`) | 100.00% | 1.3146 | 1.0537 | 2.9956 | 4.9795 | 0.5528 | 9.1385 |
| **MoveIt2 IK** (`/compute_ik`) | 99.50% | 14.6991 | 3.1735 | 29.1491 | 166.0979 | 0.8306 | 1007.9373 |
| **Custom IK** (`/ik_solve`) | 100.00% | 1.4831 | 1.2675 | 2.9603 | 4.5759 | 0.6851 | 8.1299 |

## Key Findings

1. **FK Performance:** Compare the Custom analytical/numerical formulation computation speed against MoveIt2's setup.
2. **IK Robustness & Latency:** Custom numerical Jacobian (Damped Least Squares) method success rates vs. OMPL/MoveIt2's solver configuration.
