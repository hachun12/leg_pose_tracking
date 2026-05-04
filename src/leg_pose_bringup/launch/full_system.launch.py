from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    front_openpose = Node(
        package="leg_pose_openpose",
        executable="openpose_node",
        name="front_openpose_node",
        parameters=[
            {
                "camera_name": "front",
                "image_topic": "/front_camera/color/image_rect",
                "keypoints_topic": "/leg_pose/front/keypoints_2d",
                "overlay_topic": "/leg_pose/front/skeleton_overlay",
            }
        ],
    )
    side_openpose = Node(
        package="leg_pose_openpose",
        executable="openpose_node",
        name="side_openpose_node",
        parameters=[
            {
                "camera_name": "side",
                "image_topic": "/side_camera/color/image_rect",
                "keypoints_topic": "/leg_pose/side/keypoints_2d",
                "overlay_topic": "/leg_pose/side/skeleton_overlay",
            }
        ],
    )
    return LaunchDescription(
        [
            Node(package="leg_pose_camera", executable="zed_status_node", name="zed_status_node"),
            front_openpose,
            side_openpose,
            Node(package="leg_pose_fusion", executable="keypoint_fusion_node", name="keypoint_fusion_node"),
            Node(package="leg_pose_fusion", executable="angle_estimator_node", name="angle_estimator_node"),
            Node(package="leg_pose_gui", executable="topic_monitor_node", name="topic_monitor_node"),
            Node(package="leg_pose_gui", executable="status_gui_node", name="status_gui_node"),
        ]
    )
