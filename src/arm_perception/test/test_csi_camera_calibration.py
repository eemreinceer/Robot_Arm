import pytest

from arm_perception.csi_camera_node import (
    argus_gstreamer_pipeline,
    libcamera_gstreamer_pipeline,
    camera_info_from_calibration,
    validate_camera_info_contract,
    validate_flip_method,
)


def calibration_contract(**overrides):
    data = {
        'image_width': 640,
        'image_height': 480,
        'flip_method': 2,
        'distortion_model': 'plumb_bob',
        'camera_matrix': {'data': [520.0, 0.0, 319.5, 0.0, 520.0, 239.5, 0.0, 0.0, 1.0]},
        'distortion_coefficients': {'data': [0.1, -0.2, 0.0, 0.0, 0.05]},
        'rectification_matrix': {'data': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]},
        'projection_matrix': {'data': [520.0, 0.0, 319.5, 0.0, 0.0, 520.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]},
    }
    data.update(overrides)
    return data


def test_argus_pipeline_applies_selected_flip_method():
    pipeline = argus_gstreamer_pipeline(0, 1640, 1232, 640, 480, 30, 2)
    assert 'nvvidconv flip-method=2' in pipeline


def test_camera_info_contract_accepts_matching_resolution_and_flip():
    assert validate_camera_info_contract(
        calibration_contract(), 640, 480, 2) == 2


def test_matching_calibration_builds_camera_info():
    info = camera_info_from_calibration(calibration_contract(), 640, 480, 2)
    assert (info.width, info.height) == (640, 480)
    assert info.k[0] == 520.0
    assert info.d == pytest.approx([0.1, -0.2, 0.0, 0.0, 0.05])


@pytest.mark.parametrize('value', [-1, 4, True, 2.5, '2', 'sideways'])
def test_flip_method_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        validate_flip_method(value)


def test_camera_info_contract_rejects_missing_flip():
    data = calibration_contract()
    del data['flip_method']
    with pytest.raises(ValueError, match='flip_method icermiyor'):
        validate_camera_info_contract(data, 640, 480, 2)


def test_camera_info_contract_rejects_flip_mismatch():
    with pytest.raises(ValueError, match='uyusmuyor|kamera flip_method'):
        validate_camera_info_contract(calibration_contract(), 640, 480, 0)


def test_camera_info_contract_rejects_invalid_calibration_flip():
    with pytest.raises(ValueError, match='tam sayi'):
        validate_camera_info_contract(
            calibration_contract(flip_method=2.5), 640, 480, 2)


def test_camera_info_contract_rejects_resolution_mismatch():
    with pytest.raises(ValueError, match='640x480'):
        validate_camera_info_contract(calibration_contract(), 480, 640, 2)


# --- Raspberry Pi 5 / libcamera arka ucu ---------------------------------

def test_libcamera_pipeline_uses_libcamerasrc_and_scales_to_output():
    pipeline = libcamera_gstreamer_pipeline(1640, 1232, 640, 480, 30, 0)
    assert pipeline.startswith('libcamerasrc !')
    assert 'width=(int)1640' in pipeline and 'height=(int)1232' in pipeline
    assert 'videoscale' in pipeline
    assert 'width=(int)640' in pipeline and 'height=(int)480' in pipeline
    assert 'format=(string)BGR' in pipeline
    assert 'nvarguscamerasrc' not in pipeline and 'nvvidconv' not in pipeline


def test_libcamera_pipeline_requests_nv12_from_the_source():
    """format alani bos birakilirsa libcamera 1640x1232'de ISP'yi atlayip ham
    SBGGR akisi seciyor -- 2026-08-19'da Pi 5 + IMX219'da olculdu, pipeline
    videoflip/videoconvert asamasinda 'Internal data stream error' ile
    aciliyordu. NV12 acikca istenince ISP devreye giriyor.
    """
    pipeline = libcamera_gstreamer_pipeline(1640, 1232, 640, 480, 30, 0)
    assert 'format=(string)NV12' in pipeline
    before_videoflip = pipeline.split('videoflip')[0]
    assert 'format=(string)NV12' in before_videoflip


def test_libcamera_pipeline_selects_camera_by_name_when_given():
    assert 'camera-name=/base/i2c0/imx219@10' in libcamera_gstreamer_pipeline(
        1640, 1232, 640, 480, 30, 0, '/base/i2c0/imx219@10')
    assert 'camera-name' not in libcamera_gstreamer_pipeline(
        1640, 1232, 640, 480, 30, 0)


@pytest.mark.parametrize('flip_method,expected', [
    (0, 'method=none'),
    (1, 'method=counterclockwise'),
    (2, 'method=rotate-180'),
    (3, 'method=clockwise'),
])
def test_libcamera_pipeline_maps_nvvidconv_flip_to_videoflip(flip_method, expected):
    """flip_method nvvidconv anlamini KORUR; kalibrasyon sozlesmesi ona bagli.

    Sayilar iki elementte ayni seyi ifade etmedigi icin esleme test edilir --
    sessiz bir kayma, kalibrasyonu gecerli gorunen ama yonu yanlis bir
    goruntuye baglar.
    """
    assert expected in libcamera_gstreamer_pipeline(
        1640, 1232, 640, 480, 30, flip_method)


@pytest.mark.parametrize('value', [-1, 4, 'x', None])
def test_libcamera_pipeline_rejects_invalid_flip(value):
    with pytest.raises(ValueError):
        libcamera_gstreamer_pipeline(1640, 1232, 640, 480, 30, value)
