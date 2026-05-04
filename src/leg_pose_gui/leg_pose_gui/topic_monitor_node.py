from collections import deque

import rclpy
from rclpy.node import Node

from leg_pose_msgs.msg import LegJointAngles, LegTrackingStatus


class TopicMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("topic_monitor_node")
        self.declare_parameter("window_s", 2.0)
        self._angle_times = deque()
        self._publisher = self.create_publisher(LegTrackingStatus, "/leg_pose/tracking_status", 5)
        self.create_subscription(LegJointAngles, "/leg_pose/joint_angles", self._on_angles, 10)
        self.create_timer(0.5, self._publish_status)

    def _on_angles(self, _msg: LegJointAngles) -> None:
        self._angle_times.append(self.get_clock().now().nanoseconds * 1e-9)

    def _publish_status(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        window_s = float(self.get_parameter("window_s").value)
        while self._angle_times and now - self._angle_times[0] > window_s:
            self._angle_times.popleft()

        msg = LegTrackingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_publish_hz = len(self._angle_times) / window_s
        if not self._angle_times:
            msg.warnings.append("/leg_pose/joint_angles not publishing")
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TopicMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
