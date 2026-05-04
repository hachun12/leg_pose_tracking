from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="leg_pose_gui",
                executable="topic_monitor_node",
                name="topic_monitor_node",
            ),
            Node(
                package="leg_pose_gui",
                executable="qt_gui_node",
                name="qt_gui_node",
            ),
        ]
    )
