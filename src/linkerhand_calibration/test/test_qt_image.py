from types import SimpleNamespace

from linkerhand_calibration.calibration_gui import qimage_from_ros_image


def make_image(encoding='bgr8', width=2, height=1, data=None):
    channels = 1 if encoding == 'mono8' else 3
    step = width * channels
    return SimpleNamespace(
        encoding=encoding,
        width=width,
        height=height,
        step=step,
        data=data if data is not None else bytes(step * height),
    )


def test_bgr_ros_image_converts_without_cv_bridge():
    image = qimage_from_ros_image(make_image(data=bytes([0, 0, 255, 0, 255, 0])))

    assert image.width() == 2
    assert image.height() == 1
    assert image.pixelColor(0, 0).getRgb()[:3] == (255, 0, 0)
    assert image.pixelColor(1, 0).getRgb()[:3] == (0, 255, 0)


def test_invalid_ros_image_data_is_rejected():
    try:
        qimage_from_ros_image(make_image(data=b'\x00'))
    except ValueError as error:
        assert '数据不足' in str(error)
    else:
        raise AssertionError('数据不足的 ROS 图像应被拒绝')
