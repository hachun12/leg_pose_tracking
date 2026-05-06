from leg_pose_fusion.keypoint_fusion_node import _append_synthesized_toe
from leg_pose_msgs.msg import LegKeypoint3D, LegKeypoints3D


def _point(name, x, y, z, confidence=0.8, valid=True):
    keypoint = LegKeypoint3D()
    keypoint.name = name
    keypoint.point.x = x
    keypoint.point.y = y
    keypoint.point.z = z
    keypoint.confidence = confidence
    keypoint.valid = valid
    return keypoint


def test_append_synthesized_toe_from_body25_toes():
    msg = LegKeypoints3D()
    msg.keypoints.append(_point("big_toe", 1.0, 2.0, 3.0, confidence=0.7))
    msg.keypoints.append(_point("small_toe", 3.0, 4.0, 5.0, confidence=0.6))

    _append_synthesized_toe(msg)

    toe = msg.keypoints[-1]
    assert toe.name == "toe"
    assert toe.point.x == 2.0
    assert toe.point.y == 3.0
    assert toe.point.z == 4.0
    assert toe.confidence == 0.6
    assert toe.valid


def test_append_synthesized_toe_requires_valid_big_and_small_toe():
    msg = LegKeypoints3D()
    msg.keypoints.append(_point("big_toe", 1.0, 2.0, 3.0, valid=True))
    msg.keypoints.append(_point("small_toe", 3.0, 4.0, 5.0, valid=False))

    _append_synthesized_toe(msg)

    assert msg.keypoints[-1].name == "toe"
    assert not msg.keypoints[-1].valid
