from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from leg_pose_msgs.msg import LegJointAngles, LegTrackingStatus


class ZedStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("zed_status_node")
        self.declare_parameter("front_image_topic", "/front_camera/color/image_rect")
        self.declare_parameter("side_image_topic", "/side_camera/color/image_rect")
        self.declare_parameter("connection_timeout_s", 1.0)
        self.declare_parameter("angle_topic", "/leg_pose/joint_angles")
        self.declare_parameter("angle_window_s", 2.0)
        self._front_stamp = None
        self._side_stamp = None
        self._angle_times = deque()
        self._publisher = self.create_publisher(LegTrackingStatus, "/leg_pose/tracking_status", 5)
        front_topic = self.get_parameter("front_image_topic").value
        side_topic = self.get_parameter("side_image_topic").value
        if front_topic:
            self.create_subscription(Image, front_topic, self._on_front_image, 10)
        if side_topic:
            self.create_subscription(Image, side_topic, self._on_side_image, 10)
        self.create_subscription(
            LegJointAngles,
            self.get_parameter("angle_topic").value,
            self._on_angles,
            10,
        )
        self._timer = self.create_timer(1.0, self._publish_status)

    def _on_front_image(self, _msg: Image) -> None:
        self._front_stamp = self._now_seconds()

    def _on_side_image(self, _msg: Image) -> None:
        self._side_stamp = self._now_seconds()

    def _on_angles(self, _msg: LegJointAngles) -> None:
        self._angle_times.append(self._now_seconds())

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_status(self) -> None:
        now = self._now_seconds()
        timeout_s = float(self.get_parameter("connection_timeout_s").value)
        angle_window_s = float(self.get_parameter("angle_window_s").value)
        while self._angle_times and now - self._angle_times[0] > angle_window_s:
            self._angle_times.popleft()
        front_topic = self.get_parameter("front_image_topic").value
        side_topic = self.get_parameter("side_image_topic").value
        front_connected = (
            bool(front_topic)
            and self._front_stamp is not None
            and now - self._front_stamp <= timeout_s
        )
        side_connected = (
            bool(side_topic)
            and self._side_stamp is not None
            and now - self._side_stamp <= timeout_s
        )
        msg = LegTrackingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_publish_hz = len(self._angle_times) / angle_window_s
        msg.front_camera_connected = front_connected
        msg.side_camera_connected = side_connected
        msg.tf_available = False
        msg.calibration_loaded = False
        if self._front_stamp is not None and self._side_stamp is not None:
            msg.camera_timestamp_delta_ms = abs(self._front_stamp - self._side_stamp) * 1000.0
        warnings = []
        if front_topic and not front_connected:
            warnings.append("front ZED 2i image stream missing")
        if side_topic and not side_connected:
            warnings.append("side ZED 2i image stream missing")
        if not self._angle_times:
            warnings.append("/leg_pose/joint_angles not publishing")
        msg.warnings = warnings
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
