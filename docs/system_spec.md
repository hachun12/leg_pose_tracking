# ROS2 Leg Pose Tracking System Spec

## 系統摘要

本系統在 ROS2 架構下接收正面與側面深度相機影像，透過 OpenPose 偵測下肢 2D 骨架，結合深度與雙相機外參估測 3D keypoints，計算腿部 4 個角度，並提供 GUI 與 ROS topic 給控制程式使用。

## 硬體

硬體確定採用兩台 ZED 2i 深度相機。

相機配置：

- `front_camera`：位於腿部正面，用於左右/內外翻與足部橫向資訊。
- `side_camera`：位於腿部側面，用於 hip flexion、knee flexion、ankle dorsiflexion。

## 軟體架構

ROS2 packages：

- `leg_pose_msgs`
- `leg_pose_camera`
- `leg_pose_openpose`
- `leg_pose_fusion`
- `leg_pose_gui`
- `leg_pose_bringup`

## ROS Frames

建議 frame：

- `world` 或 `map`
- `leg_tracking_frame`：角度計算共同座標系。
- `front_camera_link`
- `front_camera_color_optical_frame`
- `side_camera_link`
- `side_camera_color_optical_frame`
- `pelvis_frame`：可選，由 neutral pose 或 keypoints 推定。

所有 3D keypoints 應轉換到 `leg_tracking_frame` 後再計算角度。

## Topics

### Inputs

- `/front_camera/color/image_rect`
- `/front_camera/aligned_depth_to_color/image_raw`
- `/front_camera/color/camera_info`
- `/side_camera/color/image_rect`
- `/side_camera/aligned_depth_to_color/image_raw`
- `/side_camera/color/camera_info`
- `/tf`
- `/tf_static`

實際 topic 名稱依 `zed-ros2-wrapper` 輸出調整，透過 launch remapping 統一為上述介面。

### Intermediate Outputs

- `/leg_pose/front/keypoints_2d`
- `/leg_pose/side/keypoints_2d`
- `/leg_pose/keypoints_3d`
- `/leg_pose/front/skeleton_overlay`
- `/leg_pose/side/skeleton_overlay`
- `/leg_pose/debug/fusion_status`

### Final Output

- `/leg_pose/joint_angles`

QoS：

- Camera image 使用 sensor data QoS。
- Angle output 使用 reliable 或 best effort 需由控制端需求決定；初版建議 reliable + depth 5。

## Message Spec

### `LegJointAngles.msg`

```text
std_msgs/Header header
string side
float32 hip_flexion_extension_deg
float32 knee_flexion_deg
float32 ankle_dorsi_plantar_deg
float32 ankle_inversion_eversion_deg
float32 hip_confidence
float32 knee_confidence
float32 ankle_dorsi_confidence
float32 ankle_inversion_confidence
bool hip_valid
bool knee_valid
bool ankle_dorsi_valid
bool ankle_inversion_valid
```

`side` 使用 `left`、`right` 或 `unknown`。

### `LegKeypoint2D.msg`

```text
string name
float32 x
float32 y
float32 confidence
```

### `LegKeypoints2D.msg`

```text
std_msgs/Header header
string camera_name
LegKeypoint2D[] keypoints
```

### `LegKeypoint3D.msg`

```text
string name
geometry_msgs/Point point
float32 confidence
bool valid
```

### `LegKeypoints3D.msg`

```text
std_msgs/Header header
string frame_id
string side
LegKeypoint3D[] keypoints
```

## Angle Definitions

### Hip Flexion/Extension

使用 hip 到 knee 的大腿向量，投影到 sagittal plane。以 neutral calibration pose 的大腿方向或 `leg_tracking_frame` 的 vertical axis 作為 0 度參考。屈曲為正，伸展為負。

### Knee Flexion

向量：

- thigh：hip -> knee
- shank：ankle -> knee 或 knee -> ankle，需在實作中固定方向。

建議輸出為 0 度代表膝完全伸直，正值代表屈膝。

### Ankle Dorsiflexion/Plantarflexion

向量：

- shank：knee -> ankle
- foot：heel -> toe 或 ankle -> toe。

投影到 sagittal plane 後計算。背屈為正，蹠屈為負。

### Ankle Inversion/Eversion

使用 foot medial/lateral 或 toe/heel keypoints 在 frontal plane 的方向變化。若 OpenPose foot keypoints 不穩，需考慮加入 AprilTag、鞋面 marker、IMU 或改用 learning-based foot pose estimator。

## Node Spec

### `openpose_node`

Responsibilities：

- Subscribe RGB images。
- Run OpenPose BODY_25。
- Publish 2D keypoints and overlay image。
- Use OpenPose Python bindings when available; otherwise keep the node alive with passthrough overlays and empty keypoints for integration testing。

Parameters：

- `model_folder`
- `net_resolution`
- `gpu_id`
- `min_keypoint_confidence`
- `body_model`
- `model_folder`
- `net_resolution`
- `gpu_id`
- `enable_inference`

### `keypoint_fusion_node`

Responsibilities：

- Subscribe 2D keypoints, depth, camera info, TF。
- Back-project 2D keypoints to 3D。
- Fuse front/side estimates。
- Publish 3D keypoints。
- Initial implementation supports aligned-depth back-projection and view-local 3D output; TF transform into `leg_tracking_frame` is the next required step。

Parameters：

- `depth_window_px`
- `min_depth_m`
- `max_depth_m`
- `fusion_strategy`
- `target_frame`

### `angle_estimator_node`

Responsibilities：

- Subscribe 3D keypoints。
- Filter keypoints or angles。
- Compute 4 angles。
- Publish `LegJointAngles`。

Parameters：

- `neutral_pose_file`
- `filter_type`
- `filter_cutoff_hz`
- `angle_limits_deg`
- `min_angle_confidence`
- `apply_neutral_offsets`

Services：

- `/leg_pose/capture_neutral_pose` (`std_srvs/Trigger`) captures the latest raw angles as neutral offsets and optionally persists them to `neutral_pose_file`。

### `leg_pose_gui`

Responsibilities：

- Subscribe overlay images and joint angles。
- Display two camera views and angle widgets。
- Display OpenPose skeleton overlay, keypoint labels, confidence colors, and tracking state for front and side views。
- Display four joint angles with value, unit, valid flag, confidence, warning state, and 5-10 second trend plot。
- Display tracking quality metrics: OpenPose FPS, angle publish rate, end-to-end latency, front/side timestamp difference, lost keypoint count, and depth invalid ratio。
- Display keypoint debug details for hip/knee/ankle/heel/toe, including confidence, validity, and selected fusion source。
- Display system status: front/side ZED 2i connection, TF availability, calibration file loaded, and `leg_tracking_frame` transform status。
- Provide neutral pose calibration control with capture button, timestamp, validity state, and save-to-config behavior。
- Provide rosbag record/replay controls; replay mode must use the same widgets as live mode。
- Provide topic monitor for `/leg_pose/joint_angles`: last publish time, publish Hz, message preview, and subscriber count when available。
- Show warnings for low confidence, invalid depth, camera desync, angle out of range, OpenPose FPS drop, and TF lookup failure。
- Keep GUI responsive when camera streams, angle topic, or debug topics are temporarily missing。

Suggested layout：

- Left: front and side camera views with skeleton overlays。
- Right top: four angle readouts with valid/confidence state。
- Right middle: angle trend plot and selectable angle channels。
- Right bottom: tracking quality, calibration, record/replay, topic monitor, and warnings。

## Launch Profiles

- `zed_tracking.launch.py`
- `replay_tracking.launch.py`
- `demo.launch.py`
- `gui.launch.py`
- `qt_gui.launch.py`
- `record_tracking.launch.py`
- `full_system.launch.py`

## Configuration Files

- `config/cameras/zed.yaml`
- `config/calibration/front_to_tracking.yaml`
- `config/calibration/side_to_tracking.yaml`
- `config/angle_estimator.yaml`
- `config/openpose.yaml`

## Validation

Unit tests：

- 2D depth back-projection。
- TF transform consistency。
- Vector angle calculation。
- Invalid keypoint propagation。

Integration tests：

- Replay rosbag and assert `/leg_pose/joint_angles` publish rate。
- Compare known static poses against expected approximate angles。
- Verify GUI starts and subscribes without blocking compute nodes。

Performance targets：

- Publish rate：minimum 15 Hz，target 30 Hz。
- End-to-end latency：量測後依控制端需求設定上限。
- Angle jitter：依應用決定，初版以 2-5 degrees RMS 為調校目標。
