import struct

from sensor_msgs.msg import CameraInfo, Image

from leg_pose_fusion.depth_projection import back_project, depth_at, depth_window_median


def test_depth_at_reads_32fc1_meters():
    image = Image()
    image.width = 2
    image.height = 1
    image.encoding = "32FC1"
    image.step = 8
    image.data = struct.pack("<ff", 1.25, 2.5)
    assert depth_at(image, 1, 0) == 2.5


def test_depth_at_reads_16uc1_millimeters():
    image = Image()
    image.width = 2
    image.height = 1
    image.encoding = "16UC1"
    image.step = 4
    image.data = struct.pack("<HH", 0, 1250)
    assert depth_at(image, 0, 0) is None
    assert depth_at(image, 1, 0) == 1.25


def test_depth_window_ignores_invalid_values():
    image = Image()
    image.width = 3
    image.height = 1
    image.encoding = "16UC1"
    image.step = 6
    image.data = struct.pack("<HHH", 0, 1000, 2000)
    assert depth_window_median(image, 1, 0, 1) == 1.5


def test_back_project_uses_camera_intrinsics():
    info = CameraInfo()
    info.k = [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]
    point = back_project(info, 60.0, 20.0, 2.0)
    assert point.x == 0.2
    assert point.y == -0.4
    assert point.z == 2.0
