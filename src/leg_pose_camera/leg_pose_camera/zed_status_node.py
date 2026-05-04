import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from leg_pose_msgs.msg import LegTrackingStatus


class ZedStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("zed_status_node")
        self.declare_parameter("front_image_topic", "/front_camera/color/image_rect")
        self.declare_parameter("side_image_topic", "/side_camera/color/image_rect")
        self.declare_parameter("connection_timeout_s", 1.0)
        self._front_stamp = None
        self._side_stamp = None
        self._publisher = self.create_publisher(LegTrackingStatus, "/leg_pose/tracking_status", 5)
        self.create_subscription(
            Image,
            self.get_parameter("front_image_topic").value,
            self._on_front_image,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("side_image_topic").value,
            self._on_side_image,
            10,
        )
        self._timer = self.create_timer(1.0, self._publish_status)

    def _on_front_image(self, msg: Image) -> None:
        self._front_stamp = self._stamp_to_seconds(msg)

    def _on_side_image(self, msg: Image) -> None:
        self._side_stamp = self._stamp_to_seconds(msg)

    def _stamp_to_seconds(self, msg: Image) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _publish_status(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        timeout_s = float(self.get_parameter("connection_timeout_s").value)
        front_connected = self._front_stamp is not None and now - self._front_stamp <= timeout_s
        side_connected = self._side_stamp is not None and now - self._side_stamp <= timeout_s
        msg = LegTrackingStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.front_camera_connected = front_connected
        msg.side_camera_connected = side_connected
        msg.tf_available = False
        msg.calibration_loaded = False
        if self._front_stamp is not None and self._side_stamp is not None:
            msg.camera_timestamp_delta_ms = abs(self._front_stamp - self._side_stamp) * 1000.0
        warnings = []
        if not front_connected:
            warnings.append("front ZED 2i image stream missing")
        if not side_connected:
            warnings.append("side ZED 2i image stream missing")
        msg.warnings = warnings
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
