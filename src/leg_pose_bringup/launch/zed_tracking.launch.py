from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="leg_pose_camera", executable="zed_status_node", name="zed_status_node"),
        ]
    )
