import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Vec3:
    x: float
    y: float
    z: float

    def __sub__(self, other: "Vec3") -> "Vec3":
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def dot(self, other: "Vec3") -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def norm(self) -> float:
        return math.sqrt(self.dot(self))


def angle_deg(a: Vec3, b: Vec3) -> float:
    denom = a.norm() * b.norm()
    if denom <= 1e-9:
        raise ValueError("cannot compute angle for near-zero vector")
    cosine = max(-1.0, min(1.0, a.dot(b) / denom))
    return math.degrees(math.acos(cosine))


def project_xz(v: Vec3) -> Vec3:
    return Vec3(v.x, 0.0, v.z)


def project_yz(v: Vec3) -> Vec3:
    return Vec3(0.0, v.y, v.z)


def knee_flexion_deg(hip: Vec3, knee: Vec3, ankle: Vec3) -> float:
    thigh = hip - knee
    shank = ankle - knee
    return 180.0 - angle_deg(thigh, shank)


def hip_flexion_extension_deg(hip: Vec3, knee: Vec3) -> float:
    thigh = knee - hip
    vertical_down = Vec3(0.0, 0.0, -1.0)
    angle = angle_deg(project_xz(thigh), vertical_down)
    return math.copysign(angle, thigh.x)


def ankle_dorsi_plantar_deg(knee: Vec3, ankle: Vec3, heel: Vec3, toe: Vec3) -> float:
    shank = ankle - knee
    foot = toe - heel
    return angle_deg(project_xz(shank), project_xz(foot)) - 90.0


def ankle_inversion_eversion_deg(heel: Vec3, toe: Vec3) -> float:
    foot = toe - heel
    vertical = Vec3(0.0, 0.0, 1.0)
    return math.copysign(angle_deg(project_yz(foot), vertical) - 90.0, foot.y)
