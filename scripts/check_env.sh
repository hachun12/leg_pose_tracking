#!/usr/bin/env bash
set -u

echo "== OS =="
lsb_release -a 2>/dev/null || true

echo
echo "== ROS =="
echo "ROS_DISTRO=${ROS_DISTRO:-<not sourced>}"
if command -v ros2 >/dev/null; then
  echo "ros2: $(command -v ros2)"
else
  echo "ros2: missing"
fi
if command -v colcon >/dev/null; then
  echo "colcon: $(command -v colcon)"
else
  echo "colcon: missing"
fi

echo
echo "== GPU =="
if command -v nvidia-smi >/dev/null; then
  nvidia-smi
else
  echo "nvidia-smi: missing"
fi

echo
echo "== Python Modules =="
python3 - <<'PY'
mods = ["cv2", "cv_bridge", "PySide6", "pyqtgraph", "yaml", "openpose", "pyopenpose"]
for mod in mods:
    try:
        __import__(mod)
        print(f"{mod}: ok")
    except Exception as exc:
        print(f"{mod}: {type(exc).__name__}: {exc}")
PY

echo
echo "== ROS Packages =="
if command -v ros2 >/dev/null; then
  for pkg in cv_bridge image_transport tf2_ros tf2_geometry_msgs zed_wrapper zed_msgs; do
    if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
      echo "$pkg: ok"
    else
      echo "$pkg: missing"
    fi
  done
fi
