# Codex Handoff

Read this file first when continuing the project.

## Project Goal

Build a ROS 2 Humble system that tracks side-view leg angles using one ZED 2i camera and OpenPose:

- Hip flexion/extension.
- Knee flexion.
- Ankle dorsiflexion/plantarflexion.

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

Current target machine baseline:

- NVIDIA RTX 4080 detected with driver 580.142.
- CUDA 13.0 is the default toolkit in interactive shells.
- ROS 2 Humble is installed and the workspace builds.
- ZED SDK 5.3.0 is installed; `pyzed.sl` imports successfully.
- `zed-ros2-wrapper` is present under `src/zed-ros2-wrapper` and builds.
- Front ZED 2i serial `34108459` is available as `/dev/video0`.
- OpenPose Python binding is built under `external/openpose` with BODY_25, CUDA 13.0, and cuDNN 9.21.1.
- The active application scope was simplified to one side-view camera. The same physical ZED can be launched as `side_zed`; the former front-camera path is no longer required.
- Safety gate is now part of the live path. `angle_estimator_node` publishes `/leg_pose/joint_angles_raw`; `angle_safety_gate_node` publishes final `/leg_pose/joint_angles`.
- Live launch defaults to safety gate OFF, which passes complete valid raw detections through. If the current raw detection is invalid, `angle_safety_gate_node` republishes the last complete valid detected angle with frame id `hold_last_valid`; before any valid detection exists it suppresses invalid output. Turning safety gate ON via `/leg_pose/set_safety_gate_enabled` or the Qt GUI publishes the configured fixed safe angles instead.
- Default safe angles are in `src/leg_pose_bringup/config/safety_gate.yaml`. The Qt GUI reads and writes those live node parameters through `/angle_safety_gate_node/get_parameters` and `/angle_safety_gate_node/set_parameters`, so the safe angles can be edited at runtime without code changes.

The synthetic demo is still the regression path that should stay working after every change.

## Validation Already Performed

```bash
colcon build --symlink-install
colcon test --packages-select leg_pose_fusion leg_pose_gui --event-handlers console_direct+
colcon test --packages-select leg_pose_openpose --event-handlers console_direct+
```

Target-machine validation:

```bash
ZED_Explorer -a
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i serial_number:=34108459 camera_name:=front_zed
ros2 topic hz /front_zed/zed_node/rgb/color/rect/image
ros2 topic hz /front_zed/zed_node/depth/depth_registered
```

Observed front camera topics:

```text
/front_zed/zed_node/rgb/color/rect/image
/front_zed/zed_node/rgb/color/rect/camera_info
/front_zed/zed_node/depth/depth_registered
/front_zed/zed_node/depth/depth_registered/camera_info
```

OpenPose Python/cuDNN validation:

```bash
cd external/openpose
python3 - <<'PY'
from openpose import pyopenpose as op
w = op.WrapperPython()
w.configure({"model_folder": "models/", "model_pose": "BODY_25", "net_resolution": "-1x368", "num_gpu_start": 0})
w.start()
print("openpose cudnn configure/start ok")
PY
```

Single front-ZED OpenPose validation:

```bash
ros2 run leg_pose_openpose openpose_node --ros-args \
  -r __node:=front_openpose_node \
  -p camera_name:=front \
  -p side:=right \
  -p image_topic:=/front_zed/zed_node/rgb/color/rect/image \
  -p keypoints_topic:=/leg_pose/front/keypoints_2d \
  -p overlay_topic:=/leg_pose/front/skeleton_overlay \
  -p model_folder:=/home/r2-public-pc/workspace/leg_pose_tracking/external/openpose/models \
  -p body_model:=BODY_25 \
  -p net_resolution:=-1x368 \
  -p gpu_id:=0 \
  -p min_keypoint_confidence:=0.35
```

Observed:

```text
/front_zed/zed_node/rgb/color/rect/image: about 48-55 Hz
/front_zed/zed_node/depth/depth_registered: about 43-54 Hz
/leg_pose/front/keypoints_2d: about 33 Hz
/leg_pose/front/skeleton_overlay: publishing, about 10-25 Hz during the short check
```

One sampled keypoint message had the correct frame and camera fields, but no keypoints because no suitable person/leg was visible above the confidence threshold:

```text
frame_id: front_zed_left_camera_frame_optical
camera_name: front
side: right
keypoints: []
```

After a person stood in the front ZED view, OpenPose detected the full right lower-leg set at `min_keypoint_confidence:=0.35`:

```text
hip confidence: 0.62
knee confidence: 0.80
ankle confidence: 0.71
big_toe confidence: 0.75
small_toe confidence: 0.72
heel confidence: 0.68
/leg_pose/front/keypoints_2d: about 34 Hz
```

Single-camera depth back-projection also worked when `keypoint_fusion_node` was run with `target_frame:=front_zed_left_camera_frame_optical`:

```text
/leg_pose/keypoints_3d: about 33 Hz
hip/knee/ankle/big_toe/small_toe/heel: valid true
depth range in the sampled message: about 1.25-1.33 m
```

The first angle-estimator check published `/leg_pose/joint_angles`, but all angle fields were invalid because the estimator requires a semantic `toe` keypoint while BODY_25 provides `big_toe` and `small_toe`. This was fixed in `keypoint_fusion_node` by appending `toe` as the midpoint of `big_toe` and `small_toe`, with confidence set to the lower of the two toe confidences. Rebuild passed, but the post-fix live angle recheck was not completed because the execution environment stopped allowing new escalated ROS process launches.

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

1. Run the simplified side-view launch:
   `ros2 launch leg_pose_bringup side_tracking.launch.py`
2. Verify `/leg_pose/joint_angles` with a person visible from the side.
3. Use the Qt GUI or service to switch the safety gate only when fixed safe angles are needed:
   `ros2 service call /leg_pose/set_safety_gate_enabled std_srvs/srv/SetBool "{data: true}"`
   `ros2 service call /leg_pose/set_safety_gate_enabled std_srvs/srv/SetBool "{data: false}"`
4. Edit safe angle defaults in `src/leg_pose_bringup/config/safety_gate.yaml`, or tune them live from the Qt GUI's `Safety Gate Angles` controls.
5. Tune:
   - `min_keypoint_confidence`
   - `depth_window_px`
   - `min_depth_m`
   - `max_depth_m`
   - `filter_cutoff_hz`
    - angle sign conventions
6. Capture neutral pose and verify zeroed angles.
7. Record rosbag sessions and replay them for GUI/algorithm tuning.

## Coding Rules For Continuation

- Keep OpenPose-specific objects inside `leg_pose_openpose`.
- The OpenPose Python API requires `op.VectorDatum([datum])` for `emplaceAndPop`; a plain Python list fails with this build.
- BODY_25 emits `big_toe` and `small_toe`; `keypoint_fusion_node` synthesizes `toe` for the angle estimator.
- Side-view mode intentionally leaves ankle inversion/eversion unused. The ROS message fields remain for compatibility, but GUI/console output hides them and the estimator does not compute them.
- In multi-person scenes, `leg_pose_openpose` selects the person with the largest confident lower-body keypoint box as a practical nearest-person proxy.
- Do not bypass `angle_safety_gate_node` for robot control. Downstream machines should subscribe to `/leg_pose/joint_angles`, not `/leg_pose/joint_angles_raw`.
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
