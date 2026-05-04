# Message Reference

## Final Angle Topic

Topic: `/leg_pose/joint_angles`

Message: `leg_pose_msgs/LegJointAngles`

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

Rules:

- `header.frame_id` should be `leg_tracking_frame`.
- `side` should be `left`, `right`, or `unknown`.
- Confidence should be normalized to 0.0-1.0.
- Invalid angles may retain the last numeric value only if the corresponding `*_valid` flag is false.

## Keypoint Topics

Topic: `/leg_pose/front/keypoints_2d`

Topic: `/leg_pose/side/keypoints_2d`

Message: `leg_pose_msgs/LegKeypoints2D`

```text
std_msgs/Header header
string camera_name
LegKeypoint2D[] keypoints
```

Topic: `/leg_pose/keypoints_3d`

Message: `leg_pose_msgs/LegKeypoints3D`

```text
std_msgs/Header header
string frame_id
string side
LegKeypoint3D[] keypoints
```
