from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="leg_pose_fusion",
                executable="synthetic_keypoints_node",
                name="synthetic_keypoints_node",
            ),
            Node(
                package="leg_pose_fusion",
                executable="angle_estimator_node",
                name="angle_estimator_node",
            ),
            Node(
                package="leg_pose_gui",
                executable="topic_monitor_node",
                name="topic_monitor_node",
            ),
            Node(
                package="leg_pose_gui",
                executable="status_gui_node",
                name="status_gui_node",
            ),
        ]
    )
