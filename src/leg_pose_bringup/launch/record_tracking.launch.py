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
                    "/front_camera/color/image_rect",
                    "/front_camera/aligned_depth_to_color/image_raw",
                    "/front_camera/color/camera_info",
                    "/side_camera/color/image_rect",
                    "/side_camera/aligned_depth_to_color/image_raw",
                    "/side_camera/color/camera_info",
                    "/leg_pose/front/keypoints_2d",
                    "/leg_pose/side/keypoints_2d",
                    "/leg_pose/keypoints_3d",
                    "/leg_pose/joint_angles",
                    "/leg_pose/tracking_status",
                    "/tf",
                    "/tf_static",
                ],
                output="screen",
            ),
        ]
    )
