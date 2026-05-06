import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from leg_pose_msgs.msg import LegKeypoint2D, LegKeypoints2D

from .backend import OpenPoseBackend


class OpenPoseNode(Node):
    """OpenPose adapter node."""

    def __init__(self) -> None:
        super().__init__("openpose_node")
        self.declare_parameter("camera_name", "front")
        self.declare_parameter("side", "unknown")
        self.declare_parameter("image_topic", "/front_camera/color/image_rect")
        self.declare_parameter("keypoints_topic", "/leg_pose/front/keypoints_2d")
        self.declare_parameter("overlay_topic", "/leg_pose/front/skeleton_overlay")
        self.declare_parameter("min_keypoint_confidence", 0.35)
        self.declare_parameter("body_model", "BODY_25")
        self.declare_parameter("model_folder", "")
        self.declare_parameter("net_resolution", "-1x368")
        self.declare_parameter("gpu_id", 0)
        self.declare_parameter("enable_inference", True)

        self._camera_name = self.get_parameter("camera_name").value
        self._side = self.get_parameter("side").value
        self._bridge = CvBridge()
        self._backend = self._create_backend()
        self._keypoints_pub = self.create_publisher(
            LegKeypoints2D,
            self.get_parameter("keypoints_topic").value,
            10,
        )
        self._overlay_pub = self.create_publisher(
            Image,
            self.get_parameter("overlay_topic").value,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._on_image,
            10,
        )

    def _create_backend(self):
        if not bool(self.get_parameter("enable_inference").value):
            self.get_logger().warn("OpenPose inference disabled; publishing passthrough overlays")
            return None
        try:
            return OpenPoseBackend(
                model_folder=self.get_parameter("model_folder").value,
                body_model=self.get_parameter("body_model").value,
                net_resolution=self.get_parameter("net_resolution").value,
                gpu_id=int(self.get_parameter("gpu_id").value),
                min_confidence=float(self.get_parameter("min_keypoint_confidence").value),
                side=self._side,
            )
        except RuntimeError as exc:
            self.get_logger().warn(f"{exc}; publishing passthrough overlays")
            return None

    def _on_image(self, image: Image) -> None:
        keypoints = LegKeypoints2D()
        keypoints.header = image.header
        keypoints.camera_name = self._camera_name
        keypoints.side = self._side
        if self._backend is None:
            self._keypoints_pub.publish(keypoints)
            self._overlay_pub.publish(image)
            return

        cv_image = self._bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        result = self._backend.infer(cv_image)
        for detected in result.keypoints:
            keypoint = LegKeypoint2D()
            keypoint.name = detected.name
            keypoint.x = detected.x
            keypoint.y = detected.y
            keypoint.confidence = detected.confidence
            keypoints.keypoints.append(keypoint)
        self._keypoints_pub.publish(keypoints)
        overlay = self._bridge.cv2_to_imgmsg(result.overlay, encoding="bgr8")
        overlay.header = image.header
        self._overlay_pub.publish(overlay)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
