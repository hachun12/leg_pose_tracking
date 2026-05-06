import os

import rclpy
from rclpy.parameter import Parameter

from leg_pose_fusion.angle_safety_gate_node import AngleSafetyGateNode
from leg_pose_msgs.msg import LegJointAngles


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_safe_angles_use_configured_specific_angles():
    os.environ["ROS_LOG_DIR"] = "/tmp/leg_pose_test_ros_logs"
    rclpy.init()
    node = AngleSafetyGateNode()
    try:
        node.set_parameters(
            [
                Parameter(
                    "safe_hip_flexion_extension_deg",
                    Parameter.Type.DOUBLE,
                    11.0,
                ),
                Parameter(
                    "safe_knee_flexion_deg",
                    Parameter.Type.DOUBLE,
                    22.0,
                ),
                Parameter(
                    "safe_ankle_dorsi_plantar_deg",
                    Parameter.Type.DOUBLE,
                    -3.0,
                ),
            ]
        )

        msg = node._safe_angles()

        assert msg.hip_flexion_extension_deg == 11.0
        assert msg.knee_flexion_deg == 22.0
        assert msg.ankle_dorsi_plantar_deg == -3.0
        assert msg.hip_valid
        assert msg.knee_valid
        assert msg.ankle_dorsi_valid
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_safety_gate_off_passes_valid_detected_angles_through():
    os.environ["ROS_LOG_DIR"] = "/tmp/leg_pose_test_ros_logs"
    rclpy.init()
    node = AngleSafetyGateNode()
    publisher = FakePublisher()
    node._publisher = publisher
    try:
        msg = LegJointAngles()
        msg.hip_flexion_extension_deg = 13.0
        msg.knee_flexion_deg = -4.0
        msg.hip_valid = True
        msg.knee_valid = True
        msg.ankle_dorsi_valid = True

        node._safety_gate_enabled = False
        node._on_angles(msg)

        assert publisher.messages == [msg]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_safety_gate_off_holds_last_valid_when_detected_angles_are_invalid():
    os.environ["ROS_LOG_DIR"] = "/tmp/leg_pose_test_ros_logs"
    rclpy.init()
    node = AngleSafetyGateNode()
    publisher = FakePublisher()
    node._publisher = publisher
    try:
        valid = LegJointAngles()
        valid.hip_flexion_extension_deg = 13.0
        valid.knee_flexion_deg = 22.0
        valid.ankle_dorsi_plantar_deg = -3.0
        valid.hip_valid = True
        valid.knee_valid = True
        valid.ankle_dorsi_valid = True

        invalid = LegJointAngles()
        invalid.hip_flexion_extension_deg = 0.0
        invalid.knee_flexion_deg = 0.0
        invalid.ankle_dorsi_plantar_deg = 0.0
        invalid.hip_valid = False
        invalid.knee_valid = False
        invalid.ankle_dorsi_valid = False

        node._safety_gate_enabled = False
        node._on_angles(valid)
        node._on_angles(invalid)

        assert len(publisher.messages) == 2
        assert publisher.messages[1].header.frame_id == "hold_last_valid"
        assert publisher.messages[1].hip_flexion_extension_deg == 13.0
        assert publisher.messages[1].knee_flexion_deg == 22.0
        assert publisher.messages[1].ankle_dorsi_plantar_deg == -3.0
        assert publisher.messages[1].hip_valid
        assert publisher.messages[1].knee_valid
        assert publisher.messages[1].ankle_dorsi_valid
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_safety_gate_off_suppresses_invalid_until_first_valid_angles():
    os.environ["ROS_LOG_DIR"] = "/tmp/leg_pose_test_ros_logs"
    rclpy.init()
    node = AngleSafetyGateNode()
    publisher = FakePublisher()
    node._publisher = publisher
    try:
        invalid = LegJointAngles()

        node._safety_gate_enabled = False
        node._on_angles(invalid)

        assert publisher.messages == []
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_safety_gate_on_blocks_detected_angles_and_publishes_safe_angles():
    os.environ["ROS_LOG_DIR"] = "/tmp/leg_pose_test_ros_logs"
    rclpy.init()
    node = AngleSafetyGateNode()
    publisher = FakePublisher()
    node._publisher = publisher
    try:
        msg = LegJointAngles()
        msg.hip_flexion_extension_deg = 13.0

        node._safety_gate_enabled = True
        node._on_angles(msg)
        node._publish_safe_if_enabled()

        assert len(publisher.messages) == 1
        assert publisher.messages[0].header.frame_id == "safety_gate"
        assert publisher.messages[0].hip_flexion_extension_deg == 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
