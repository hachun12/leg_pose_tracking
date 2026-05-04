---
name: ros2-leg-pose-tracking
description: Build, extend, debug, or plan a ROS2 leg joint angle tracking system using OpenPose with two ZED 2i front/side cameras, including 2D/3D keypoint fusion, hip/knee/ankle angle estimation, GUI visualization, ROS messages, launch files, calibration, rosbag replay, and validation workflows.
---

# ROS2 Leg Pose Tracking

Use this skill when implementing or modifying the leg pose tracking project. Optimize for ROS2-native boundaries: messages, nodes, parameters, launch files, TF frames, and rosbag replay.

## Workflow

1. Read `docs/system_spec.md` and `docs/project_plan.md` before making architectural changes.
2. Preserve the package split unless the user explicitly asks to simplify:
   - `leg_pose_msgs`
   - `leg_pose_camera`
   - `leg_pose_openpose`
   - `leg_pose_fusion`
   - `leg_pose_gui`
   - `leg_pose_bringup`
3. Keep OpenPose-specific data structures inside `leg_pose_openpose`; downstream packages should consume project messages.
4. Compute angles only from 3D keypoints in a common frame, not directly from raw 2D pixels.
5. Publish angle confidence and validity along with angle values.
6. Add or update rosbag replay paths for features that depend on hardware.

## Implementation Defaults

- Prefer ROS2 Humble or Jazzy patterns.
- Prefer Python nodes for rapid OpenPose, fusion, and GUI prototypes unless the user requires C++.
- Use `sensor_data` QoS for images and depth.
- Use TF2 for camera-to-tracking-frame transforms.
- Use BODY_25 OpenPose keypoints when foot/ankle estimation is required.
- Keep GPU inference out of GUI process.
- Make launch files target two ZED 2i cameras through `zed-ros2-wrapper`, remapping driver topics into the project topic contract.

## Angle Rules

- Hip flexion/extension: thigh vector projected to sagittal plane, neutral pose or vertical axis as reference.
- Knee flexion: 0 degrees means extended; positive means flexion.
- Ankle dorsiflexion/plantarflexion: shank and foot vectors projected to sagittal plane.
- Ankle inversion/eversion: use front-view foot orientation; mark invalid when foot keypoints are unreliable.

Always document sign convention when changing angle math.

## Validation Checklist

Run or create tests for:

- Angle math on synthetic 3D points.
- Depth back-projection from keypoint pixels.
- TF transform into `leg_tracking_frame`.
- Invalid depth and low-confidence keypoint propagation.
- Rosbag replay publish rate for `/leg_pose/joint_angles`.

For GUI changes, verify:

- Front and side views render independently.
- Skeleton overlay appears on both views.
- Four angles update without blocking image display.
- Invalid/confidence state is visible.
- Angle trend plot shows recent values without resizing the layout.
- Tracking quality metrics are visible: OpenPose FPS, angle publish rate, latency, camera timestamp delta, lost keypoints, and depth invalid ratio.
- Neutral pose calibration can be triggered and reports timestamp/validity.
- Record/replay controls are present and replay mode uses the same display path as live mode.
- Topic monitor shows `/leg_pose/joint_angles` freshness and publish Hz.
- Warnings are surfaced for low confidence, invalid depth, camera desync, angle limits, FPS drop, and TF lookup failure.

## References

- Read `references/messages.md` when adding or changing ROS messages.
- Read `references/node_contracts.md` when adding nodes, topics, parameters, or launch files.
