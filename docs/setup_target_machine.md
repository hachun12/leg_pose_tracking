# Target Machine Setup

This document is for the GPU workstation that will run two ZED 2i cameras and OpenPose.

## Current Development Baseline

- Ubuntu: 22.04 Jammy
- ROS 2: Humble
- Project path used during development: `/home/hachun/Workspace/leg_pose_tracking`
- GPU is not required for the synthetic demo, but is required for practical ZED 2i + OpenPose operation.

## 1. Install Base ROS Tools

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  python3-pip \
  python3-yaml
```

Initialize rosdep if needed:

```bash
sudo rosdep init || true
rosdep update
```

Install GUI/runtime dependencies:

```bash
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-vision-opencv \
  ros-humble-rviz2

pip3 install --user PySide6 pyqtgraph numpy
```

## 2. Verify NVIDIA GPU

```bash
nvidia-smi
```

Do not continue to ZED/OpenPose runtime validation until `nvidia-smi` works.

## 3. Install ZED SDK

Install the ZED SDK for Ubuntu 22.04 from Stereolabs.

```bash
sudo apt update
sudo apt install -y zstd
chmod +x ZED_SDK_Ubuntu22_cuda*.run
./ZED_SDK_Ubuntu22_cuda*.run
```

Reboot after installation. Then verify each ZED 2i with ZED Explorer or ZED Depth Viewer.

## 4. Clone And Build This Workspace

```bash
mkdir -p ~/Workspace
cd ~/Workspace
git clone <GITHUB_REPO_URL> leg_pose_tracking
cd leg_pose_tracking

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Verify the portable demo first:

```bash
export ROS_LOG_DIR=$PWD/log/ros
ros2 launch leg_pose_bringup demo.launch.py
```

Expected output:

```text
angles side=right ... | angle_hz=30.0
```

## 5. Install ZED ROS 2 Wrapper

Clone the wrapper into the same workspace:

```bash
cd ~/Workspace/leg_pose_tracking/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git

cd ~/Workspace/leg_pose_tracking
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Verify a single ZED 2i first:

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Then inspect topics:

```bash
ros2 topic list | grep zed
```

Map the actual ZED wrapper topics to this project contract in `src/leg_pose_bringup/config/cameras/zed.yaml` and launch files:

- `/front_camera/color/image_rect`
- `/front_camera/aligned_depth_to_color/image_raw`
- `/front_camera/color/camera_info`
- `/side_camera/color/image_rect`
- `/side_camera/aligned_depth_to_color/image_raw`
- `/side_camera/color/camera_info`

## 6. Install OpenPose Python API

Build OpenPose from source with Python enabled:

```bash
sudo apt install -y build-essential cmake git python3-dev python3-pip
pip3 install --user numpy opencv-python

git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git ~/Workspace/openpose
cd ~/Workspace/openpose
git submodule update --init --recursive
mkdir -p build
cd build
cmake -DBUILD_PYTHON=ON ..
make -j$(nproc)
```

Expose the Python binding. The exact path depends on your OpenPose build, but commonly:

```bash
export PYTHONPATH=$PYTHONPATH:~/Workspace/openpose/build/python
```

Verify:

```bash
python3 -c "from openpose import pyopenpose as op; print('openpose ok')"
```

Set `model_folder` in `src/leg_pose_bringup/config/openpose.yaml`.

## 7. Run The System

Terminal 1:

```bash
source install/setup.bash
export ROS_LOG_DIR=$PWD/log/ros
ros2 launch leg_pose_bringup full_system.launch.py
```

Terminal 2:

```bash
source install/setup.bash
ros2 launch leg_pose_bringup qt_gui.launch.py
```

Capture neutral pose after the subject is in the neutral reference pose:

```bash
ros2 service call /leg_pose/capture_neutral_pose std_srvs/srv/Trigger {}
```

Record a session:

```bash
ros2 launch leg_pose_bringup record_tracking.launch.py output:=bags/session_001
```

## 8. First Real-Hardware Checklist

- `nvidia-smi` works.
- Both ZED 2i cameras are visible in ZED tools.
- ZED ROS 2 wrapper publishes RGB, aligned depth, and camera info for both cameras.
- Static TF from both ZED optical frames to `leg_tracking_frame` is available.
- OpenPose Python import works.
- `ros2 topic hz /leg_pose/joint_angles` is stable.
- Qt GUI shows image overlays, angle values, angle trend, and warnings.

