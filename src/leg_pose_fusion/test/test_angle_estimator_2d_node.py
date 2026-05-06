import os

import rclpy

from leg_pose_fusion.angle_estimator_2d_node import AngleEstimator2DNode
from leg_pose_msgs.msg import LegKeypoint2D, LegKeypoints2D


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _keypoint(name, x, y, confidence=0.9):
    keypoint = LegKeypoint2D()
    keypoint.name = name
    keypoint.x = float(x)
    keypoint.y = float(y)
    keypoint.confidence = confidence
    return keypoint


def test_2d_estimator_computes_angles_from_image_xy():
    os.environ["ROS_LOG_DIR"] = "/tmp/leg_pose_test_ros_logs"
    rclpy.init()
    node = AngleEstimator2DNode()
    publisher = FakePublisher()
    node._publisher = publisher
    try:
        msg = LegKeypoints2D()
        msg.side = "right"
        msg.keypoints.extend(
            [
                _keypoint("hip", 100.0, 100.0),
                _keypoint("knee", 100.0, 200.0),
                _keypoint("ankle", 100.0, 300.0),
                _keypoint("heel", 80.0, 320.0),
                _keypoint("big_toe", 120.0, 320.0),
                _keypoint("small_toe", 120.0, 320.0),
            ]
        )

        node._on_keypoints(msg)

        assert len(publisher.messages) == 1
        out = publisher.messages[0]
        assert abs(out.hip_flexion_extension_deg) < 1e-6
        assert abs(out.knee_flexion_deg) < 1e-6
        assert abs(out.ankle_dorsi_plantar_deg) < 1e-6
        assert out.hip_valid
        assert out.knee_valid
        assert out.ankle_dorsi_valid
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_2d_estimator_marks_low_confidence_angles_invalid():
    os.environ["ROS_LOG_DIR"] = "/tmp/leg_pose_test_ros_logs"
    rclpy.init()
    node = AngleEstimator2DNode()
    publisher = FakePublisher()
    node._publisher = publisher
    try:
        msg = LegKeypoints2D()
        msg.side = "right"
        msg.keypoints.extend(
            [
                _keypoint("hip", 100.0, 100.0),
                _keypoint("knee", 100.0, 200.0),
                _keypoint("ankle", 100.0, 300.0, confidence=0.1),
                _keypoint("heel", 80.0, 320.0),
                _keypoint("big_toe", 120.0, 320.0),
                _keypoint("small_toe", 120.0, 320.0),
            ]
        )

        node._on_keypoints(msg)

        out = publisher.messages[0]
        assert out.hip_valid
        assert not out.knee_valid
        assert not out.ankle_dorsi_valid
    finally:
        node.destroy_node()
        rclpy.shutdown()
