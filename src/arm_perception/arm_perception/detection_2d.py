"""Build timestamp-aligned ``vision_msgs`` detections from YOLO results."""

from __future__ import annotations

from typing import Iterable, Mapping, Sequence

from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)


def _as_float_list(values) -> list[float]:
    """Convert a tensor/array/list bbox to four plain floats."""
    if hasattr(values, 'cpu'):
        values = values.cpu()
    if hasattr(values, 'tolist'):
        values = values.tolist()
    return [float(value) for value in values]


def detection_array_from_yolo(header, results: Sequence) -> Detection2DArray:
    """
    Convert the first Ultralytics result into ``Detection2DArray``.

    The output header is copied from the source image. Consumers can therefore
    synchronize 2D boxes, compressed camera frames and 3D ``ObjectArray``
    messages without guessing from wall-clock arrival time.
    """
    output = Detection2DArray()
    output.header = header
    if not results:
        return output

    result = results[0]
    boxes: Iterable = result.boxes if result.boxes is not None else ()
    names: Mapping = result.names
    for index, box in enumerate(boxes):
        bounds = _as_float_list(box.xyxy[0])
        if len(bounds) != 4:
            continue
        x_min, y_min, x_max, y_max = bounds
        if x_max <= x_min or y_max <= y_min:
            continue

        class_id = int(box.cls[0].item())
        class_name = str(names[class_id])
        confidence = float(box.conf[0].item())

        detected = Detection2D()
        detected.header = header
        detected.id = f'{class_name}_{index}'
        detected.bbox.center.position.x = (x_min + x_max) / 2.0
        detected.bbox.center.position.y = (y_min + y_max) / 2.0
        detected.bbox.center.theta = 0.0
        detected.bbox.size_x = x_max - x_min
        detected.bbox.size_y = y_max - y_min

        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_name
        hypothesis.hypothesis.score = confidence
        detected.results.append(hypothesis)
        output.detections.append(detected)
    return output


def detection_to_dict(detected: Detection2D) -> dict:
    """Return the stable wire representation used by the web gateway."""
    center = detected.bbox.center.position
    hypothesis = detected.results[0].hypothesis if detected.results else None
    return {
        'id': detected.id,
        'className': hypothesis.class_id if hypothesis else 'unknown',
        'confidence': float(hypothesis.score) if hypothesis else 0.0,
        'bbox': {
            'centerX': float(center.x),
            'centerY': float(center.y),
            'width': float(detected.bbox.size_x),
            'height': float(detected.bbox.size_y),
        },
    }
