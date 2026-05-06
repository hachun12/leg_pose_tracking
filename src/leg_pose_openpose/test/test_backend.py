import sys
import types

import numpy as np

from leg_pose_openpose.backend import OpenPoseBackend


def test_backend_uses_openpose_vector_datum(monkeypatch):
    class FakeDatum:
        def __init__(self):
            self.cvInputData = None
            self.cvOutputData = None
            self.poseKeypoints = None

    class FakeVectorDatum(list):
        pass

    class FakeWrapperPython:
        last_instance = None

        def __init__(self):
            self.processed_arg = None
            FakeWrapperPython.last_instance = self

        def configure(self, params):
            self.params = params

        def start(self):
            self.started = True

        def emplaceAndPop(self, datums):
            self.processed_arg = datums
            datums[0].cvOutputData = datums[0].cvInputData
            datums[0].poseKeypoints = np.empty((0, 25, 3), dtype=np.float32)
            return True

    fake_pyopenpose = types.SimpleNamespace(
        Datum=FakeDatum,
        VectorDatum=FakeVectorDatum,
        WrapperPython=FakeWrapperPython,
    )
    fake_openpose = types.SimpleNamespace(pyopenpose=fake_pyopenpose)
    monkeypatch.setitem(sys.modules, "openpose", fake_openpose)
    monkeypatch.setitem(sys.modules, "openpose.pyopenpose", fake_pyopenpose)

    backend = OpenPoseBackend(
        model_folder="/tmp/models",
        body_model="BODY_25",
        net_resolution="-1x368",
        gpu_id=0,
        min_confidence=0.35,
        side="right",
    )

    result = backend.infer(np.zeros((4, 4, 3), dtype=np.uint8))

    assert isinstance(FakeWrapperPython.last_instance.processed_arg, FakeVectorDatum)
    assert result.keypoints == []


def test_backend_selects_largest_lower_body_person(monkeypatch):
    class FakeDatum:
        def __init__(self):
            self.cvInputData = None
            self.cvOutputData = None
            self.poseKeypoints = None

    class FakeVectorDatum(list):
        pass

    class FakeWrapperPython:
        def configure(self, _params):
            pass

        def start(self):
            pass

        def emplaceAndPop(self, datums):
            small_person = np.zeros((25, 3), dtype=np.float32)
            large_person = np.zeros((25, 3), dtype=np.float32)
            for index, x, y in (
                (9, 10.0, 10.0),
                (10, 20.0, 20.0),
                (11, 20.0, 30.0),
                (22, 25.0, 35.0),
                (23, 23.0, 35.0),
                (24, 18.0, 34.0),
            ):
                small_person[index] = [x, y, 0.9]
            for index, x, y in (
                (9, 100.0, 100.0),
                (10, 180.0, 260.0),
                (11, 190.0, 420.0),
                (22, 230.0, 520.0),
                (23, 210.0, 525.0),
                (24, 170.0, 500.0),
            ):
                large_person[index] = [x, y, 0.8]
            datums[0].cvOutputData = datums[0].cvInputData
            datums[0].poseKeypoints = np.asarray([small_person, large_person])
            return True

    fake_pyopenpose = types.SimpleNamespace(
        Datum=FakeDatum,
        VectorDatum=FakeVectorDatum,
        WrapperPython=FakeWrapperPython,
    )
    fake_openpose = types.SimpleNamespace(pyopenpose=fake_pyopenpose)
    monkeypatch.setitem(sys.modules, "openpose", fake_openpose)
    monkeypatch.setitem(sys.modules, "openpose.pyopenpose", fake_pyopenpose)

    backend = OpenPoseBackend(
        model_folder="/tmp/models",
        body_model="BODY_25",
        net_resolution="-1x368",
        gpu_id=0,
        min_confidence=0.35,
        side="right",
    )

    result = backend.infer(np.zeros((4, 4, 3), dtype=np.uint8))

    assert result.keypoints[0].name == "hip"
    assert result.keypoints[0].x == 100.0
