# Legacy IK model artifacts

Trained `.pt`, `.pth` and `.onnx` files are intentionally not distributed in
this public portfolio. They are generated artifacts and may inherit licensing
or dataset restrictions that are not represented by the repository license.

The historical benchmark reports remain available under
`src/arm_tests/benchmark_results/`. To reproduce the legacy experiment, place
the training dataset outside Git, run `arm_ml.train`, and record the producing
commit, dataset version, training command, dependency versions, model hash and
redistribution terms before sharing the result.

Expected local runtime filenames are `ik_net.pt` and
`ik_net_scripted.pt`. Both are ignored by Git.
