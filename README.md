# ROS2 Leg Pose Tracking

ROS2 Humble project for tracking sagittal-plane leg motion with one side-view ZED 2i camera, OpenPose keypoints, ZED depth back-projection, angle publishing, and GUI/status visualization.

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

## Launch Side-View Tracking

```bash
export ROS_LOG_DIR=$PWD/log/ros
source install/setup.bash
ros2 launch leg_pose_bringup side_tracking.launch.py
```

The default camera serial is `34108459`. Override it if needed:

```bash
ros2 launch leg_pose_bringup side_tracking.launch.py serial_number:=YOUR_ZED_SERIAL
```

The live system publishes:

```text
/leg_pose/side/keypoints_2d
/leg_pose/side/skeleton_overlay
/leg_pose/keypoints_3d
/leg_pose/joint_angles_raw
/leg_pose/joint_angles
```

The tracked angles are:

- Hip flexion/extension.
- Knee flexion.
- Ankle dorsiflexion/plantarflexion.

Ankle inversion/eversion is intentionally not tracked in the side-view-only setup.

## Safety Gate

`/leg_pose/joint_angles_raw` contains the detected angles from vision.
`/leg_pose/joint_angles` is the safety-gated output for downstream machines.

In live tracking, the safety gate is OFF by default, so `/leg_pose/joint_angles`
passes through complete valid detections from `/leg_pose/joint_angles_raw`.
Turn the safety gate ON to publish configured safe angles instead.

Default safe angles live in `src/leg_pose_bringup/config/safety_gate.yaml`, so
they can be changed without touching code:

```text
safe_hip_flexion_extension_deg: 0.0
safe_knee_flexion_deg: 0.0
safe_ankle_dorsi_plantar_deg: 0.0
```

After launch, the Qt GUI shows these safe angles in `Safety Gate Angles`.
Editing the three degree fields updates `angle_safety_gate_node` parameters
immediately; when Safety Gate is ON, the next safe-angle publish uses the new
values.

When Safety Gate is OFF, invalid detections do not fall back to the configured
safe angle. The gate holds and republishes the last complete valid detected
angle instead. Before the first valid detection, invalid `0 deg` messages are
suppressed rather than published.

The same runtime edit can be done from ROS CLI:

```bash
ros2 param set /angle_safety_gate_node safe_hip_flexion_extension_deg 0.0
ros2 param set /angle_safety_gate_node safe_knee_flexion_deg 0.0
ros2 param set /angle_safety_gate_node safe_ankle_dorsi_plantar_deg 0.0
```

Enable or disable the safety gate:

```bash
ros2 service call /leg_pose/set_safety_gate_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /leg_pose/set_safety_gate_enabled std_srvs/srv/SetBool "{data: false}"
```

The Qt GUI has matching controls:

- `Safety Gate OFF`
- `Safety Gate ON`
- Editable `Safety Gate Angles`

The GUI shows both streams:

- Angle cards and solid plot lines show `/leg_pose/joint_angles`, the safety-gated output intended for the remote machine.
- `Detected preview` and faded plot lines show `/leg_pose/joint_angles_raw`, the vision estimate before safety gating.
- When the safety gate is OFF, complete valid detected angles pass through.
- When the safety gate is OFF and the current detection is invalid, the final output holds the last valid detected angle.
- When the safety gate is ON, the final output is the fixed safe angle command.

If multiple people are visible, the OpenPose adapter selects the person with the largest confident lower-body keypoint box, which is used as a practical nearest-person proxy for side-view tracking.

Troubleshooting:

- If the side image shows a skeleton but final angle cards stay at `0.0 deg`, check `Detected preview`.
- If `Detected preview` is invalid while the safety gate is OFF, `/leg_pose/joint_angles` holds the last valid detected angles.
- `side_camera=true/false` should not flicker in the live launch; `zed_status_node` is the only tracking-status publisher there.

If the ZED wrapper is already running as `side_zed`, launch only this project's nodes:

```bash
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
angles side=right hip=15.1 knee=11.1 ankle_dp=-24.2 | angle_hz=30.0
```

Launch the Qt GUI against live or demo topics:

```bash
export ROS_LOG_DIR=$PWD/log/ros
source install/setup.bash
ros2 launch leg_pose_bringup qt_gui.launch.py
```

`side_tracking.launch.py` starts the Qt GUI automatically. For a console-only fallback:

```bash
source install/setup.bash
ros2 launch leg_pose_bringup gui.launch.py
```

The Qt GUI uses PySide6 when available and falls back to PyQt5. No `pyqtgraph` dependency is required.

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

1. Run `side_tracking.launch.py` and verify `/leg_pose/joint_angles`.
2. Tune neutral-pose offsets, low-pass cutoff, and angle sign conventions with real side-view data.
3. Record side-view bags for repeatable GUI and algorithm tuning.
