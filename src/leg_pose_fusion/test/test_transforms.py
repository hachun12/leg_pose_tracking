from geometry_msgs.msg import Point, Quaternion, Transform, Vector3

from leg_pose_fusion.keypoint_fusion_node import _apply_transform


def test_apply_transform_translation_only():
    point = Point(x=1.0, y=2.0, z=3.0)
    transform = Transform(
        translation=Vector3(x=0.5, y=-1.0, z=2.0),
        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    transformed = _apply_transform(point, transform)
    assert transformed.x == 1.5
    assert transformed.y == 1.0
    assert transformed.z == 5.0


def test_apply_transform_rotates_around_z():
    point = Point(x=1.0, y=0.0, z=0.0)
    transform = Transform(
        translation=Vector3(x=0.0, y=0.0, z=0.0),
        rotation=Quaternion(x=0.0, y=0.0, z=0.70710678, w=0.70710678),
    )
    transformed = _apply_transform(point, transform)
    assert abs(transformed.x) < 1e-6
    assert abs(transformed.y - 1.0) < 1e-6
    assert transformed.z == 0.0
