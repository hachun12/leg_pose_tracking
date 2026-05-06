from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    output = LaunchConfiguration("output")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output",
                default_value="leg_pose_recording",
                description="rosbag2 output directory.",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    output,
                    "/side_zed/zed_node/rgb/color/rect/image",
                    "/side_zed/zed_node/depth/depth_registered",
                    "/side_zed/zed_node/rgb/color/rect/camera_info",
                    "/side_zed/zed_node/depth/depth_registered/camera_info",
                    "/leg_pose/side/keypoints_2d",
                    "/leg_pose/side/skeleton_overlay",
                    "/leg_pose/keypoints_3d",
                    "/leg_pose/joint_angles_raw",
                    "/leg_pose/joint_angles",
                    "/leg_pose/tracking_status",
                    "/tf",
                    "/tf_static",
                ],
                output="screen",
            ),
        ]
    )
