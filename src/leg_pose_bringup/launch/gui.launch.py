from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="leg_pose_gui",
                executable="status_gui_node",
                name="status_gui_node",
            ),
        ]
    )
