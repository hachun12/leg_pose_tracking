import rclpy
from rclpy.node import Node

from leg_pose_msgs.msg import LegJointAngles, LegTrackingStatus


class StatusGuiNode(Node):
    """Console fallback for the planned Qt GUI."""

    def __init__(self) -> None:
        super().__init__("leg_pose_status_gui_node")
        self._last_angles = None
        self._last_status = None
        self.create_subscription(LegJointAngles, "/leg_pose/joint_angles", self._on_angles, 10)
        self.create_subscription(
            LegTrackingStatus,
            "/leg_pose/tracking_status",
            self._on_status,
            10,
        )
        self.create_timer(1.0, self._render)

    def _on_angles(self, msg: LegJointAngles) -> None:
        self._last_angles = msg

    def _on_status(self, msg: LegTrackingStatus) -> None:
        self._last_status = msg

    def _render(self) -> None:
        if self._last_angles is None:
            self.get_logger().info("waiting for /leg_pose/joint_angles")
            return
        msg = self._last_angles
        status = self._status_summary()
        self.get_logger().info(
            "angles side=%s hip=%.1f knee=%.1f ankle_dp=%.1f ankle_ie=%.1f%s"
            % (
                msg.side,
                msg.hip_flexion_extension_deg,
                msg.knee_flexion_deg,
                msg.ankle_dorsi_plantar_deg,
                msg.ankle_inversion_eversion_deg,
                status,
            )
        )

    def _status_summary(self) -> str:
        if self._last_status is None:
            return ""
        status = self._last_status
        warnings = ", ".join(status.warnings)
        suffix = " | angle_hz=%.1f" % status.angle_publish_hz
        if warnings:
            suffix += " | warnings=%s" % warnings
        return suffix


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusGuiNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
