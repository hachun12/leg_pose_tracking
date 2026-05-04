import math
import struct
from dataclasses import dataclass
from statistics import median
from typing import Optional

from sensor_msgs.msg import CameraInfo, Image


@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float


def depth_at(image: Image, u: int, v: int) -> Optional[float]:
    if u < 0 or v < 0 or u >= image.width or v >= image.height:
        return None

    if image.encoding == "32FC1":
        offset = v * image.step + u * 4
        value = struct.unpack_from("<f", image.data, offset)[0]
        if not math.isfinite(value) or value <= 0.0:
            return None
        return float(value)

    if image.encoding in ("16UC1", "mono16"):
        offset = v * image.step + u * 2
        value = struct.unpack_from("<H", image.data, offset)[0]
        if value == 0:
            return None
        return float(value) / 1000.0

    return None


def depth_window_median(image: Image, u: int, v: int, radius: int) -> Optional[float]:
    values = []
    for y in range(v - radius, v + radius + 1):
        for x in range(u - radius, u + radius + 1):
            value = depth_at(image, x, y)
            if value is not None:
                values.append(value)
    if not values:
        return None
    return float(median(values))


def back_project(camera_info: CameraInfo, u: float, v: float, depth_m: float) -> Point3D:
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]
    if fx == 0.0 or fy == 0.0:
        raise ValueError("camera intrinsics fx/fy must be non-zero")
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return Point3D(x=x, y=y, z=depth_m)
