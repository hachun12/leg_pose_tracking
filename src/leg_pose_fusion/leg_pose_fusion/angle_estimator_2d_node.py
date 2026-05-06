import os

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import yaml

from leg_pose_msgs.msg import LegJointAngles, LegKeypoints2D

from .angle_filter import AngleFilterSet
from .geometry import (
    Vec3,
    ankle_dorsi_plantar_deg,
    hip_flexion_extension_deg,
    knee_flexion_deg,
)


REQUIRED_KEYPOINTS = ("hip", "knee", "ankle", "heel", "toe")


class AngleEstimator2DNode(Node):
    def __init__(self) -> None:
        super().__init__("angle_estimator_2d_node")
        self.declare_parameter("min_angle_confidence", 0.35)
        self.declare_parameter("input_topic", "/leg_pose/side/keypoints_2d")
        self.declare_parameter("output_topic", "/leg_pose/joint_angles_raw")
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
        self.create_subscription(LegKeypoints2D, input_topic, self._on_keypoints, 10)
        self.create_service(Trigger, "/leg_pose/capture_neutral_pose", self._capture_neutral)

    def _on_keypoints(self, msg: LegKeypoints2D) -> None:
        points = {keypoint.name: keypoint for keypoint in msg.keypoints}
        points = _with_synthesized_toe(points)
        out = LegJointAngles()
        out.header = msg.header
        out.header.frame_id = msg.header.frame_id
        out.side = msg.side

        missing = [name for name in REQUIRED_KEYPOINTS if name not in points]
        if missing:
            self.get_logger().debug(f"missing 2D keypoints: {missing}")
            self._publisher.publish(out)
            return

        hip, knee, ankle, heel, toe = [
            _image_point_to_side_plane(points[name]) for name in REQUIRED_KEYPOINTS
        ]
        try:
            raw_angles = {
                "hip": hip_flexion_extension_deg(hip, knee),
                "knee": knee_flexion_deg(hip, knee, ankle),
                "ankle_dorsi": ankle_dorsi_plantar_deg(knee, ankle, heel, toe),
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

        min_conf = float(self.get_parameter("min_angle_confidence").value)
        out.hip_confidence = min(points["hip"].confidence, points["knee"].confidence)
        out.knee_confidence = min(
            points["hip"].confidence,
            points["knee"].confidence,
            points["ankle"].confidence,
        )
        out.ankle_dorsi_confidence = min(
            points["knee"].confidence,
            points["ankle"].confidence,
            points["heel"].confidence,
            points["toe"].confidence,
        )
        out.hip_valid = out.hip_confidence >= min_conf
        out.knee_valid = out.knee_confidence >= min_conf
        out.ankle_dorsi_valid = out.ankle_dorsi_confidence >= min_conf
        self._publisher.publish(out)

    def _stamp_to_seconds(self, msg: LegKeypoints2D) -> float:
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


def _image_point_to_side_plane(keypoint) -> Vec3:
    return Vec3(float(keypoint.x), 0.0, -float(keypoint.y))


def _with_synthesized_toe(points):
    if "toe" in points or "big_toe" not in points or "small_toe" not in points:
        return points
    toe = type(points["big_toe"])()
    toe.name = "toe"
    toe.x = 0.5 * (points["big_toe"].x + points["small_toe"].x)
    toe.y = 0.5 * (points["big_toe"].y + points["small_toe"].y)
    toe.confidence = min(points["big_toe"].confidence, points["small_toe"].confidence)
    with_toe = dict(points)
    with_toe["toe"] = toe
    return with_toe


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AngleEstimator2DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
