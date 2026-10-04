"""2D detection contract for browser overlays and timestamp synchronization."""

from types import SimpleNamespace

from arm_perception.detection_2d import (
    detection_array_from_yolo,
    detection_to_dict,
)
import numpy as np
from std_msgs.msg import Header


class Scalar:

    def __init__(self, value):
        self.value = value

    def item(self):
        return self.value


def test_yolo_bbox_preserves_source_header_and_classification():
    header = Header()
    header.stamp.sec = 12
    header.stamp.nanosec = 34
    header.frame_id = 'camera_optical_frame'
    box = SimpleNamespace(
        xyxy=[np.asarray([10.0, 20.0, 50.0, 80.0])],
        cls=[Scalar(2)],
        conf=[Scalar(0.875)],
    )
    result = SimpleNamespace(boxes=[box], names={2: 'blue_cube'})

    output = detection_array_from_yolo(header, [result])

    assert output.header == header
    assert len(output.detections) == 1
    detected = output.detections[0]
    assert detected.header == header
    assert detected.id == 'blue_cube_0'
    assert detected.bbox.center.position.x == 30.0
    assert detected.bbox.center.position.y == 50.0
    assert detected.bbox.size_x == 40.0
    assert detected.bbox.size_y == 60.0
    assert detection_to_dict(detected) == {
        'id': 'blue_cube_0',
        'className': 'blue_cube',
        'confidence': 0.875,
        'bbox': {
            'centerX': 30.0,
            'centerY': 50.0,
            'width': 40.0,
            'height': 60.0,
        },
    }


def test_empty_and_malformed_boxes_fail_closed():
    header = Header()
    malformed = SimpleNamespace(
        xyxy=[np.asarray([10.0, 20.0, 10.0, 80.0])],
        cls=[Scalar(0)],
        conf=[Scalar(1.0)],
    )
    result = SimpleNamespace(boxes=[malformed], names={0: 'red_box'})

    assert not detection_array_from_yolo(header, []).detections
    assert not detection_array_from_yolo(header, [result]).detections
