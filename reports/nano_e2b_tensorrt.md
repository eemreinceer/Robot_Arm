# Nano E2b TensorRT Deployment Report

Date: 2026-07-05

## Result

PASS. The laptop PT -> ONNX -> Jetson Nano TensorRT FP16 engine path completed end to end.

## Laptop export

- Python: 3.12.3
- Ultralytics: 8.4.75
- Torch: 2.12.1+cu130
- ONNX: 1.22.0, opset 12
- Input: `images`, `1x3x640x640`
- Output: `output0`, `1x7x8400`
- ONNX size: 12,266,891 bytes
- ONNX SHA256: `44cf19717dcc6e4171e82da4e158c2c5541af13bee2828c3ea5e0d3cb37710a4`

## Nano engine build

- Connection: operator-supplied SSH host (`ROBOT_ARM_HOST`)
- Container: existing `humble_dev`; Torch/Ultralytics were not installed
- Device: NVIDIA Tegra X1, compute capability 5.3
- TensorRT: 8.2.1
- Command: `trtexec --onnx=yolo_arm.onnx --saveEngine=yolo_arm.engine --fp16 --workspace=2048`
- Result: exit 0, TensorRT `PASSED`
- Engine size: 11,147,443 bytes
- Engine SHA256: `d4df3f2b4291ff85e26e86df4132551e2042d61b9a646f3c039a496a3e31332a`

## Benchmark

Command: `trtexec --loadEngine=yolo_arm.engine --iterations=100 --avgRuns=100`

- Result: exit 0, 100 timed queries
- Mean GPU compute latency: 46.1372 ms
- Mean host latency: 46.6447 ms
- Mean end-to-end host latency: 46.6567 ms
- Throughput: 21.4329 qps
- Median GPU compute: 45.8462 ms
- P99 GPU compute: 65.6316 ms

The benchmark uses random input and validates engine deserialization and GPU execution. Image preprocessing, decoding, NMS, and Python integration remain a P2 decision.

ONNX and engine artifacts are covered by `src/arm_perception/.gitignore` (`models/*`). Copies are present in the laptop model directory and Nano `/workspace/artifacts/e2b/`.
