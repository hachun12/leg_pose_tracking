import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from leg_pose_msgs.msg import LegKeypoint3D, LegKeypoints3D


class SyntheticKeypointsNode(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_keypoints_node")
        self.declare_parameter("side", "right")
        self.declare_parameter("frame_id", "leg_tracking_frame")
        self.declare_parameter("publish_hz", 30.0)
        self._publisher = self.create_publisher(LegKeypoints3D, "/leg_pose/keypoints_3d", 10)
        self._start_time = self.get_clock().now()
        period = 1.0 / float(self.get_parameter("publish_hz").value)
        self.create_timer(period, self._publish_keypoints)

    def _publish_keypoints(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        knee_x = 0.20 * math.sin(elapsed)
        ankle_x = knee_x + 0.25 * math.sin(elapsed * 1.3)
        toe_x = ankle_x + 0.22

        msg = LegKeypoints3D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.frame_id = self.get_parameter("frame_id").value
        msg.side = self.get_parameter("side").value
        msg.keypoints = [
            self._keypoint("hip", 0.0, 0.0, 1.0),
            self._keypoint("knee", knee_x, 0.0, 0.45),
            self._keypoint("ankle", ankle_x, 0.0, 0.0),
            self._keypoint("heel", ankle_x - 0.08, 0.0, -0.03),
            self._keypoint("toe", toe_x, 0.03 * math.sin(elapsed * 0.7), -0.02),
        ]
        self._publisher.publish(msg)

    def _keypoint(self, name: str, x: float, y: float, z: float) -> LegKeypoint3D:
        keypoint = LegKeypoint3D()
        keypoint.name = name
        keypoint.point = Point(x=x, y=y, z=z)
        keypoint.confidence = 0.95
        keypoint.valid = True
        return keypoint


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticKeypointsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
