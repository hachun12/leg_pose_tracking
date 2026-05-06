from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    safety_gate_config = PathJoinSubstitution(
        [FindPackageShare("leg_pose_bringup"), "config", "safety_gate.yaml"]
    )
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
                parameters=[
                    {
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
