# Codex Handoff

Read this file first when continuing the project.

## Project Goal

Build a ROS 2 Humble system that tracks four leg angles using two ZED 2i cameras and OpenPose:

- Hip flexion/extension.
- Knee flexion.
- Ankle dorsiflexion/plantarflexion.
- Ankle inversion/eversion.

The system must publish `/leg_pose/joint_angles` for downstream control software and provide a GUI with camera overlays, skeleton/keypoint debug, angles, trend plots, tracking status, neutral calibration, and rosbag controls.

## Current State

The repo is intentionally portable and can run without GPU/cameras using a synthetic demo.

Implemented packages:

- `leg_pose_msgs`: custom ROS messages.
- `leg_pose_camera`: ZED status monitor scaffold.
- `leg_pose_openpose`: OpenPose adapter with optional Python backend and passthrough fallback.
- `leg_pose_fusion`: depth back-projection, TF transform, angle math, low-pass filter, neutral pose service, synthetic keypoints.
- `leg_pose_gui`: Qt GUI, console fallback, topic monitor.
- `leg_pose_bringup`: demo/full/gui/replay/record launch files and config.

Known limitation of the development laptop:

- No NVIDIA GPU.
- No `zed-ros2-wrapper`.
- No OpenPose Python binding.

Therefore the synthetic demo is the validated path on this machine.

## Validation Already Performed

```bash
colcon build --symlink-install
colcon test --packages-select leg_pose_fusion leg_pose_gui --event-handlers console_direct+
colcon test --packages-select leg_pose_openpose --event-handlers console_direct+
```

Synthetic end-to-end demo:

```bash
export ROS_LOG_DIR=$PWD/log/ros
source install/setup.bash
ros2 launch leg_pose_bringup demo.launch.py
```

Observed:

```text
angles side=right ... | angle_hz=30.0
```

## Important Files

- [README.md](../README.md)
- [docs/project_plan.md](project_plan.md)
- [docs/system_spec.md](system_spec.md)
- [docs/setup_target_machine.md](setup_target_machine.md)
- [src/leg_pose_fusion/leg_pose_fusion/angle_estimator_node.py](../src/leg_pose_fusion/leg_pose_fusion/angle_estimator_node.py)
- [src/leg_pose_fusion/leg_pose_fusion/keypoint_fusion_node.py](../src/leg_pose_fusion/leg_pose_fusion/keypoint_fusion_node.py)
- [src/leg_pose_openpose/leg_pose_openpose/openpose_node.py](../src/leg_pose_openpose/leg_pose_openpose/openpose_node.py)
- [src/leg_pose_gui/leg_pose_gui/qt_gui_node.py](../src/leg_pose_gui/leg_pose_gui/qt_gui_node.py)
- [src/leg_pose_bringup/config/cameras/zed.yaml](../src/leg_pose_bringup/config/cameras/zed.yaml)
- [src/leg_pose_bringup/config/openpose.yaml](../src/leg_pose_bringup/config/openpose.yaml)

## Next Work On GPU/ZED Machine

1. Install and validate NVIDIA driver, CUDA, ZED SDK, `zed-ros2-wrapper`, and OpenPose Python API.
2. Launch each ZED 2i independently and record actual topic names.
3. Update `zed.yaml` and launch remaps to match actual front/side camera namespaces.
4. Add static TF publishers or calibration loader for:
   - front ZED optical frame -> `leg_tracking_frame`
   - side ZED optical frame -> `leg_tracking_frame`
5. Validate `keypoint_fusion_node` with real aligned depth and camera info.
6. Set OpenPose `model_folder` and verify BODY_25 lower-body keypoints.
7. Tune:
   - `min_keypoint_confidence`
   - `depth_window_px`
   - `min_depth_m`
   - `max_depth_m`
   - `filter_cutoff_hz`
   - angle sign conventions
8. Capture neutral pose and verify zeroed angles.
9. Record rosbag sessions and replay them for GUI/algorithm tuning.

## Coding Rules For Continuation

- Keep OpenPose-specific objects inside `leg_pose_openpose`.
- Keep ZED wrapper-specific topic names in launch/config, not core math.
- Keep angle math unit-tested with synthetic 3D points.
- Do not compute final angles from raw 2D pixels.
- Propagate confidence and validity instead of hiding invalid data.
- Keep `demo.launch.py` working after every change.

## Useful Commands

Build:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Test:

```bash
colcon test --packages-select leg_pose_fusion leg_pose_gui leg_pose_openpose --event-handlers console_direct+
```

Demo:

```bash
export ROS_LOG_DIR=$PWD/log/ros
ros2 launch leg_pose_bringup demo.launch.py
```

Qt GUI:

```bash
ros2 launch leg_pose_bringup qt_gui.launch.py
```

Record:

```bash
ros2 launch leg_pose_bringup record_tracking.launch.py output:=bags/session_001
```

