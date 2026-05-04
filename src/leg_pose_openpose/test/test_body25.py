from leg_pose_openpose.body25 import keypoint_indices_for_side


def test_right_body25_lower_body_indices():
    indices = keypoint_indices_for_side("right")
    assert indices["hip"] == 9
    assert indices["knee"] == 10
    assert indices["ankle"] == 11
    assert indices["big_toe"] == 22
    assert indices["small_toe"] == 23
    assert indices["heel"] == 24


def test_left_body25_lower_body_indices():
    indices = keypoint_indices_for_side("left")
    assert indices["hip"] == 12
    assert indices["knee"] == 13
    assert indices["ankle"] == 14
    assert indices["big_toe"] == 19
    assert indices["small_toe"] == 20
    assert indices["heel"] == 21
