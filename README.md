# ROS2 Leg Pose Tracking

ROS2 Humble project for tracking four leg joint angles with two ZED 2i cameras, OpenPose keypoints, 3D keypoint fusion, angle publishing, and GUI/status visualization.

## Current Scaffold

Packages:

- `leg_pose_msgs`: custom messages for 2D/3D keypoints, joint angles, and tracking status.
- `leg_pose_camera`: ZED 2i status/camera adapter scaffold.
- `leg_pose_openpose`: OpenPose adapter with optional Python binding backend; falls back to image passthrough if OpenPose is unavailable.
- `leg_pose_fusion`: depth back-projection, TF transform support, synthetic demo keypoints, and vector-based angle math core.
- `leg_pose_gui`: Qt GUI, console status fallback, and topic monitor for angle publish rate.
- `leg_pose_bringup`: launch and config files.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch Scaffold

```bash
export ROS_LOG_DIR=$PWD/log/ros
source install/setup.bash
ros2 launch leg_pose_bringup full_system.launch.py
```

Without cameras, run the synthetic demo to validate angle publishing:

```bash
export ROS_LOG_DIR=$PWD/log/ros
source install/setup.bash
ros2 launch leg_pose_bringup demo.launch.py
```

Expected demo output includes angle values and publish rate:

```text
angles side=right hip=15.1 knee=11.1 ankle_dp=-24.2 ankle_ie=31.2 | angle_hz=30.0
```

Launch the Qt GUI against live or demo topics:

```bash
export ROS_LOG_DIR=$PWD/log/ros
source install/setup.bash
ros2 launch leg_pose_bringup qt_gui.launch.py
```

Record the tracking topics:

```bash
source install/setup.bash
ros2 launch leg_pose_bringup record_tracking.launch.py output:=bags/session_001
```

Capture neutral pose:

```bash
source install/setup.bash
ros2 service call /leg_pose/capture_neutral_pose std_srvs/srv/Trigger {}
```

## Test

```bash
source install/setup.bash
colcon test --packages-select leg_pose_fusion --event-handlers console_direct+
```

## Primary Output Topic

```bash
ros2 interface show leg_pose_msgs/msg/LegJointAngles
```

Final angle topic:

```text
/leg_pose/joint_angles
```

## Next Implementation Steps

1. Connect `zed-ros2-wrapper` topics for `front_zed2i` and `side_zed2i`.
2. Install OpenPose Python bindings on the target machine and set `model_folder`.
3. Validate BODY_25 lower-body keypoints against ZED 2i live images.
4. Tune neutral-pose offsets, low-pass cutoff, and angle sign conventions with real data.
5. Replace placeholder calibration transforms with measured front/side ZED 2i extrinsics.
