from leg_pose_fusion.geometry import Vec3, hip_flexion_extension_deg, knee_flexion_deg


def test_knee_flexion_extended_is_zero():
    hip = Vec3(0.0, 0.0, 1.0)
    knee = Vec3(0.0, 0.0, 0.0)
    ankle = Vec3(0.0, 0.0, -1.0)
    assert abs(knee_flexion_deg(hip, knee, ankle)) < 1e-6


def test_knee_flexion_right_angle():
    hip = Vec3(0.0, 0.0, 1.0)
    knee = Vec3(0.0, 0.0, 0.0)
    ankle = Vec3(1.0, 0.0, 0.0)
    assert abs(knee_flexion_deg(hip, knee, ankle) - 90.0) < 1e-6


def test_hip_flexion_positive_forward():
    hip = Vec3(0.0, 0.0, 0.0)
    knee = Vec3(1.0, 0.0, -1.0)
    assert hip_flexion_extension_deg(hip, knee) > 0.0
