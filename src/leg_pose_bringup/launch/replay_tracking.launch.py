from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bag_path = LaunchConfiguration("bag_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path", description="Path to rosbag2 directory to replay."),
            ExecuteProcess(cmd=["ros2", "bag", "play", bag_path], output="screen"),
        ]
    )
