from leg_pose_fusion.angle_filter import LowPassFilter


def test_lowpass_filter_starts_at_first_sample():
    filt = LowPassFilter(cutoff_hz=2.0)
    assert filt.update(10.0, 0.1) == 10.0


def test_lowpass_filter_moves_toward_sample():
    filt = LowPassFilter(cutoff_hz=1.0)
    filt.update(0.0, 0.1)
    value = filt.update(10.0, 0.1)
    assert 0.0 < value < 10.0


def test_lowpass_filter_reset_forgets_previous_value():
    filt = LowPassFilter(cutoff_hz=1.0)
    filt.update(0.0, 0.1)
    filt.reset()
    assert filt.update(5.0, 0.1) == 5.0
