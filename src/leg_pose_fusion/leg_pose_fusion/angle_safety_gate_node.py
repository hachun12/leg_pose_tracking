import copy

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from leg_pose_msgs.msg import LegJointAngles


class AngleSafetyGateNode(Node):
    def __init__(self) -> None:
        super().__init__("angle_safety_gate_node")
        self.declare_parameter("input_topic", "/leg_pose/joint_angles_raw")
        self.declare_parameter("output_topic", "/leg_pose/joint_angles")
        self.declare_parameter("safety_gate_enabled", False)
        self.declare_parameter("safe_publish_hz", 20.0)
        self.declare_parameter("safe_side", "right")
        self.declare_parameter("safe_hip_flexion_extension_deg", 0.0)
        self.declare_parameter("safe_knee_flexion_deg", 0.0)
        self.declare_parameter("safe_ankle_dorsi_plantar_deg", 0.0)
        self.declare_parameter("hold_last_valid_on_invalid", True)

        self._safety_gate_enabled = bool(self.get_parameter("safety_gate_enabled").value)
        self._last_valid_angles = None
        self._publisher = self.create_publisher(
            LegJointAngles,
            self.get_parameter("output_topic").value,
            10,
        )
        self.create_subscription(
            LegJointAngles,
            self.get_parameter("input_topic").value,
            self._on_angles,
            10,
        )
        self.create_service(
            SetBool,
            "/leg_pose/set_safety_gate_enabled",
            self._set_safety_gate_enabled,
        )
        safe_publish_hz = float(self.get_parameter("safe_publish_hz").value)
        period_s = 1.0 / safe_publish_hz if safe_publish_hz > 0.0 else 0.05
        self.create_timer(period_s, self._publish_safe_if_enabled)

    def _set_safety_gate_enabled(self, request: SetBool.Request, response: SetBool.Response):
        self._safety_gate_enabled = bool(request.data)
        state = "ON" if self._safety_gate_enabled else "OFF"
        response.success = True
        response.message = "Safety gate %s." % state
        return response

    def _on_angles(self, msg: LegJointAngles) -> None:
        if self._safety_gate_enabled:
            return
        if self._angles_are_valid(msg):
            self._last_valid_angles = copy.deepcopy(msg)
            self._publisher.publish(msg)
            return
        if bool(self.get_parameter("hold_last_valid_on_invalid").value):
            self._publish_last_valid()

    def _publish_last_valid(self) -> None:
        if self._last_valid_angles is None:
            return
        msg = copy.deepcopy(self._last_valid_angles)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "hold_last_valid"
        self._publisher.publish(msg)

    def _publish_safe_if_enabled(self) -> None:
        if self._safety_gate_enabled:
            self._publisher.publish(self._safe_angles())

    def _angles_are_valid(self, msg: LegJointAngles) -> bool:
        return bool(msg.hip_valid and msg.knee_valid and msg.ankle_dorsi_valid)

    def _safe_angles(self) -> LegJointAngles:
        msg = LegJointAngles()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "safety_gate"
        msg.side = self.get_parameter("safe_side").value
        msg.hip_flexion_extension_deg = float(
            self.get_parameter("safe_hip_flexion_extension_deg").value
        )
        msg.knee_flexion_deg = float(self.get_parameter("safe_knee_flexion_deg").value)
        msg.ankle_dorsi_plantar_deg = float(
            self.get_parameter("safe_ankle_dorsi_plantar_deg").value
        )
        msg.hip_confidence = 1.0
        msg.knee_confidence = 1.0
        msg.ankle_dorsi_confidence = 1.0
        msg.hip_valid = True
        msg.knee_valid = True
        msg.ankle_dorsi_valid = True
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AngleSafetyGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
