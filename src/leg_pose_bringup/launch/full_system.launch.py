from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    openpose_config = PathJoinSubstitution(
        [FindPackageShare("leg_pose_bringup"), "config", "openpose.yaml"]
    )
    safety_gate_config = PathJoinSubstitution(
        [FindPackageShare("leg_pose_bringup"), "config", "safety_gate.yaml"]
    )
    side_openpose = Node(
        package="leg_pose_openpose",
        executable="openpose_node",
        name="side_openpose_node",
        parameters=[
            openpose_config,
            {
                "camera_name": "side",
                "side": "right",
                "image_topic": "/side_zed/zed_node/rgb/color/rect/image",
                "keypoints_topic": "/leg_pose/side/keypoints_2d",
                "overlay_topic": "/leg_pose/side/skeleton_overlay",
            }
        ],
    )
    keypoint_fusion = Node(
        package="leg_pose_fusion",
        executable="keypoint_fusion_node",
        name="keypoint_fusion_node",
        parameters=[
            {
                "target_frame": "side_zed_left_camera_frame_optical",
                "side_depth_topic": "/side_zed/zed_node/depth/depth_registered",
                "side_camera_info_topic": "/side_zed/zed_node/rgb/color/rect/camera_info",
            }
        ],
    )
    return LaunchDescription(
        [
            Node(
                package="leg_pose_camera",
                executable="zed_status_node",
                name="zed_status_node",
                parameters=[
                    {
                        "front_image_topic": "",
                        "side_image_topic": "/side_zed/zed_node/rgb/color/rect/image",
                    }
                ],
            ),
            side_openpose,
            keypoint_fusion,
            Node(
                package="leg_pose_fusion",
                executable="angle_estimator_2d_node",
                name="angle_estimator_2d_node",
                parameters=[
                    {
                        "input_topic": "/leg_pose/side/keypoints_2d",
                        "output_topic": "/leg_pose/joint_angles_raw",
                    }
                ],
            ),
            Node(
                package="leg_pose_fusion",
                executable="angle_safety_gate_node",
                name="angle_safety_gate_node",
                parameters=[
                    safety_gate_config,
                    {
                        "input_topic": "/leg_pose/joint_angles_raw",
                        "output_topic": "/leg_pose/joint_angles",
                    }
                ],
            ),
            Node(
                package="leg_pose_gui",
                executable="qt_gui_node",
                name="qt_gui_node",
            ),
        ]
    )
