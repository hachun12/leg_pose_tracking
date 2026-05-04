import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from leg_pose_msgs.msg import LegKeypoint3D, LegKeypoints2D, LegKeypoints3D

from .depth_projection import back_project, depth_window_median


class CameraState:
    def __init__(self) -> None:
        self.depth = None
        self.camera_info = None


class KeypointFusionNode(Node):
    """
    Fuse front and side 2D keypoints into project 3D keypoints.

    This version back-projects keypoint pixels with aligned ZED depth and
    selects the highest-confidence valid view per keypoint. The next step is
    adding TF transforms from each optical frame into `leg_tracking_frame`.
    """

    def __init__(self) -> None:
        super().__init__("keypoint_fusion_node")
        self.declare_parameter("target_frame", "leg_tracking_frame")
        self.declare_parameter("depth_window_px", 2)
        self.declare_parameter("min_depth_m", 0.25)
        self.declare_parameter("max_depth_m", 8.0)
        self._front = CameraState()
        self._side = CameraState()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(LegKeypoints3D, "/leg_pose/keypoints_3d", 10)
        self.create_subscription(
            LegKeypoints2D,
            "/leg_pose/front/keypoints_2d",
            self._on_keypoints,
            10,
        )
        self.create_subscription(
            LegKeypoints2D,
            "/leg_pose/side/keypoints_2d",
            self._on_keypoints,
            10,
        )
        self.create_subscription(
            Image,
            "/front_camera/aligned_depth_to_color/image_raw",
            lambda msg: self._set_depth(self._front, msg),
            10,
        )
        self.create_subscription(
            Image,
            "/side_camera/aligned_depth_to_color/image_raw",
            lambda msg: self._set_depth(self._side, msg),
            10,
        )
        self.create_subscription(
            CameraInfo,
            "/front_camera/color/camera_info",
            lambda msg: self._set_camera_info(self._front, msg),
            10,
        )
        self.create_subscription(
            CameraInfo,
            "/side_camera/color/camera_info",
            lambda msg: self._set_camera_info(self._side, msg),
            10,
        )

    def _set_depth(self, state: CameraState, msg: Image) -> None:
        state.depth = msg

    def _set_camera_info(self, state: CameraState, msg: CameraInfo) -> None:
        state.camera_info = msg

    def _on_keypoints(self, msg: LegKeypoints2D) -> None:
        state = self._front if msg.camera_name == "front" else self._side
        out = LegKeypoints3D()
        out.header = msg.header
        out.frame_id = self.get_parameter("target_frame").value
        out.side = msg.side
        if state.depth is None or state.camera_info is None:
            self._publisher.publish(out)
            return

        radius = int(self.get_parameter("depth_window_px").value)
        min_depth = float(self.get_parameter("min_depth_m").value)
        max_depth = float(self.get_parameter("max_depth_m").value)
        for keypoint in msg.keypoints:
            u = int(round(keypoint.x))
            v = int(round(keypoint.y))
            depth_m = depth_window_median(state.depth, u, v, radius)
            point = LegKeypoint3D()
            point.name = keypoint.name
            point.confidence = keypoint.confidence
            point.valid = depth_m is not None and min_depth <= depth_m <= max_depth
            if point.valid:
                projected = back_project(state.camera_info, keypoint.x, keypoint.y, depth_m)
                source_frame = state.camera_info.header.frame_id or msg.header.frame_id
                transformed = self._to_target_frame(projected, source_frame)
                if transformed is None:
                    point.valid = False
                else:
                    point.point = transformed
            out.keypoints.append(point)
        self._publisher.publish(out)

    def _to_target_frame(self, projected, source_frame: str):
        target_frame = self.get_parameter("target_frame").value
        source_frame = source_frame or target_frame
        point = Point(x=projected.x, y=projected.y, z=projected.z)
        if source_frame == target_frame:
            return point
        try:
            transform = self._tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException as exc:
            self.get_logger().debug(
                f"TF unavailable from {source_frame} to {target_frame}: {exc}"
            )
            return None
        return _apply_transform(point, transform.transform)


def _apply_transform(point: Point, transform) -> Point:
    rotated = _rotate_point(point, transform.rotation)
    return Point(
        x=rotated.x + transform.translation.x,
        y=rotated.y + transform.translation.y,
        z=rotated.z + transform.translation.z,
    )


def _rotate_point(point: Point, q) -> Point:
    # Quaternion rotation expanded to avoid depending on optional Python TF helpers.
    xx = q.x * q.x
    yy = q.y * q.y
    zz = q.z * q.z
    xy = q.x * q.y
    xz = q.x * q.z
    yz = q.y * q.z
    wx = q.w * q.x
    wy = q.w * q.y
    wz = q.w * q.z
    return Point(
        x=(1.0 - 2.0 * (yy + zz)) * point.x
        + 2.0 * (xy - wz) * point.y
        + 2.0 * (xz + wy) * point.z,
        y=2.0 * (xy + wz) * point.x
        + (1.0 - 2.0 * (xx + zz)) * point.y
        + 2.0 * (yz - wx) * point.z,
        z=2.0 * (xz - wy) * point.x
        + 2.0 * (yz + wx) * point.y
        + (1.0 - 2.0 * (xx + yy)) * point.z,
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeypointFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
