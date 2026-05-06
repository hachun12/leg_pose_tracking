from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_number = LaunchConfiguration("serial_number")
    camera_model = LaunchConfiguration("camera_model")
    zed_launch = PathJoinSubstitution(
        [FindPackageShare("zed_wrapper"), "launch", "zed_camera.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_number", default_value="34108459"),
            DeclareLaunchArgument("camera_model", default_value="zed2i"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(zed_launch),
                launch_arguments={
                    "camera_model": camera_model,
                    "serial_number": serial_number,
                    "camera_name": "side_zed",
                    "node_log_type": "screen",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("leg_pose_bringup"),
                            "launch",
                            "full_system.launch.py",
                        ]
                    )
                )
            ),
        ]
    )
