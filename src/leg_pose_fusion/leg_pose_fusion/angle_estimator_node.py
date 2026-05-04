import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml

from leg_pose_msgs.msg import LegJointAngles, LegKeypoints3D

from .angle_filter import AngleFilterSet
from .geometry import (
    Vec3,
    ankle_dorsi_plantar_deg,
    ankle_inversion_eversion_deg,
    hip_flexion_extension_deg,
    knee_flexion_deg,
)


REQUIRED_KEYPOINTS = ("hip", "knee", "ankle", "heel", "toe")


class AngleEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("angle_estimator_node")
        self.declare_parameter("min_angle_confidence", 0.35)
        self.declare_parameter("input_topic", "/leg_pose/keypoints_3d")
        self.declare_parameter("output_topic", "/leg_pose/joint_angles")
        self.declare_parameter("neutral_pose_file", "")
        self.declare_parameter("filter_type", "lowpass")
        self.declare_parameter("filter_cutoff_hz", 6.0)
        self.declare_parameter("apply_neutral_offsets", True)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self._neutral_offsets = self._load_neutral_offsets()
        self._last_raw_angles = None
        self._last_stamp_s = None
        self._filters = AngleFilterSet(float(self.get_parameter("filter_cutoff_hz").value))
        self._publisher = self.create_publisher(LegJointAngles, output_topic, 5)
        self.create_subscription(LegKeypoints3D, input_topic, self._on_keypoints, 10)
        self.create_service(Trigger, "/leg_pose/capture_neutral_pose", self._capture_neutral)

    def _on_keypoints(self, msg: LegKeypoints3D) -> None:
        points = {
            kp.name: (Vec3(kp.point.x, kp.point.y, kp.point.z), kp.confidence, kp.valid)
            for kp in msg.keypoints
        }
        out = LegJointAngles()
        out.header = msg.header
        out.header.frame_id = msg.frame_id or msg.header.frame_id
        out.side = msg.side

        missing = [name for name in REQUIRED_KEYPOINTS if name not in points]
        if missing:
            self.get_logger().debug(f"missing keypoints: {missing}")
            self._publisher.publish(out)
            return

        min_conf = float(self.get_parameter("min_angle_confidence").value)
        hip, knee, ankle, heel, toe = [points[name][0] for name in REQUIRED_KEYPOINTS]
        valid = all(points[name][2] for name in REQUIRED_KEYPOINTS)

        try:
            raw_angles = {
                "hip": hip_flexion_extension_deg(hip, knee),
                "knee": knee_flexion_deg(hip, knee, ankle),
                "ankle_dorsi": ankle_dorsi_plantar_deg(knee, ankle, heel, toe),
                "ankle_inversion": ankle_inversion_eversion_deg(heel, toe),
            }
        except ValueError as exc:
            self.get_logger().debug(str(exc))
            self._publisher.publish(out)
            return
        self._last_raw_angles = raw_angles
        angles = self._apply_neutral_offsets(raw_angles)
        angles = self._apply_filters(angles, self._stamp_to_seconds(msg))
        out.hip_flexion_extension_deg = angles["hip"]
        out.knee_flexion_deg = angles["knee"]
        out.ankle_dorsi_plantar_deg = angles["ankle_dorsi"]
        out.ankle_inversion_eversion_deg = angles["ankle_inversion"]

        out.hip_confidence = min(points["hip"][1], points["knee"][1])
        out.knee_confidence = min(points["hip"][1], points["knee"][1], points["ankle"][1])
        out.ankle_dorsi_confidence = min(
            points["knee"][1],
            points["ankle"][1],
            points["heel"][1],
            points["toe"][1],
        )
        out.ankle_inversion_confidence = min(points["heel"][1], points["toe"][1])
        out.hip_valid = valid and out.hip_confidence >= min_conf
        out.knee_valid = valid and out.knee_confidence >= min_conf
        out.ankle_dorsi_valid = valid and out.ankle_dorsi_confidence >= min_conf
        out.ankle_inversion_valid = valid and out.ankle_inversion_confidence >= min_conf
        self._publisher.publish(out)

    def _stamp_to_seconds(self, msg: LegKeypoints3D) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def _apply_neutral_offsets(self, angles):
        if not bool(self.get_parameter("apply_neutral_offsets").value):
            return dict(angles)
        return {
            name: value - self._neutral_offsets.get(name, 0.0)
            for name, value in angles.items()
        }

    def _apply_filters(self, angles, stamp_s: float):
        if self.get_parameter("filter_type").value != "lowpass":
            return angles
        if self._last_stamp_s is None:
            dt_s = 0.0
        else:
            dt_s = stamp_s - self._last_stamp_s
        self._last_stamp_s = stamp_s
        return {
            "hip": self._filters.hip.update(angles["hip"], dt_s),
            "knee": self._filters.knee.update(angles["knee"], dt_s),
            "ankle_dorsi": self._filters.ankle_dorsi.update(angles["ankle_dorsi"], dt_s),
            "ankle_inversion": self._filters.ankle_inversion.update(
                angles["ankle_inversion"],
                dt_s,
            ),
        }

    def _capture_neutral(self, _request, response):
        if self._last_raw_angles is None:
            response.success = False
            response.message = "No valid raw angles available yet."
            return response
        self._neutral_offsets = dict(self._last_raw_angles)
        self._filters.reset()
        path = self.get_parameter("neutral_pose_file").value
        if path:
            self._save_neutral_offsets(path)
        response.success = True
        response.message = "Captured neutral pose offsets."
        return response

    def _load_neutral_offsets(self):
        path = self.get_parameter("neutral_pose_file").value
        defaults = {
            "hip": 0.0,
            "knee": 0.0,
            "ankle_dorsi": 0.0,
            "ankle_inversion": 0.0,
        }
        if not path or not os.path.exists(path):
            return defaults
        with open(path, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        offsets = data.get("neutral_offsets_deg", data)
        defaults.update({key: float(offsets.get(key, defaults[key])) for key in defaults})
        return defaults

    def _save_neutral_offsets(self, path: str) -> None:
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(path, "w", encoding="utf-8") as stream:
            yaml.safe_dump({"neutral_offsets_deg": self._neutral_offsets}, stream)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AngleEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
