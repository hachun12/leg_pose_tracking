# Node Contracts

## `openpose_node`

Inputs:

- RGB image.
- Camera info when overlay scaling or debug projection needs it.

Outputs:

- `leg_pose_msgs/LegKeypoints2D`.
- Skeleton overlay image.
- Empty keypoints and passthrough overlay are acceptable only when OpenPose bindings are unavailable or inference is explicitly disabled.

Parameters:

- `model_folder`
- `net_resolution`
- `gpu_id`
- `min_keypoint_confidence`
- `body_model`
- `enable_inference`

## `keypoint_fusion_node`

Inputs:

- Front and side `LegKeypoints2D`.
- Front and side aligned depth images.
- Front and side camera info.
- TF.

Outputs:

- `leg_pose_msgs/LegKeypoints3D`.
- Optional fusion debug status.

Rules:

- Back-project using camera intrinsics and aligned depth.
- Transform all points into `leg_tracking_frame`.
- Fuse views by confidence, depth validity, and known view reliability per joint.
- Do not compute final angles here unless explicitly scoped as a prototype.
- Keep `demo.launch.py` and `synthetic_keypoints_node` working so GUI and control-topic work can proceed without cameras.

## `angle_estimator_node`

Inputs:

- `leg_pose_msgs/LegKeypoints3D`.

Outputs:

- `leg_pose_msgs/LegJointAngles` on `/leg_pose/joint_angles`.

Rules:

- Keep sign convention stable and documented.
- Propagate invalid keypoints to invalid angle flags.
- Clamp only after computing raw values; avoid hiding upstream geometry issues during debug.

## `leg_pose_gui`

Inputs:

- Front skeleton overlay image.
- Side skeleton overlay image.
- `LegJointAngles`.
- Optional keypoint 2D/3D debug topics.
- Optional fusion/debug status topics.
- Optional node diagnostics or performance metrics.

Rules:

- Keep GUI separate from OpenPose inference.
- GUI must tolerate missing camera streams or missing angle topic.
- Show four angles and validity/confidence state.
- Show recent angle trend plots, preferably 5-10 seconds of history.
- Show tracking quality: OpenPose FPS, angle publish rate, end-to-end latency, front/side timestamp difference, lost keypoints, and depth invalid ratio.
- Show keypoint debug details for hip/knee/ankle/heel/toe, including confidence, validity, and fusion source.
- Show system state for front/side ZED 2i connection, TF availability, calibration loaded, and `leg_tracking_frame`.
- Provide neutral pose capture control and persist the resulting calibration when requested.
- Provide rosbag record/replay controls without changing the visualization path between live and replay.
- Provide `/leg_pose/joint_angles` topic freshness, publish Hz, message preview, and subscriber count when available.
- Surface warnings for low confidence, invalid depth, camera desync, angle out of range, OpenPose FPS drop, and TF lookup failure.
