from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .body25 import keypoint_indices_for_side


@dataclass(frozen=True)
class DetectedKeypoint:
    name: str
    x: float
    y: float
    confidence: float


@dataclass(frozen=True)
class OpenPoseResult:
    keypoints: List[DetectedKeypoint]
    overlay: np.ndarray


class OpenPoseBackend:
    def __init__(
        self,
        model_folder: str,
        body_model: str,
        net_resolution: str,
        gpu_id: int,
        min_confidence: float,
        side: str,
    ) -> None:
        self._min_confidence = min_confidence
        self._side = side
        self._op_wrapper = None
        self._op_datum_class = None
        self._op_vector_datum_class = None
        try:
            from openpose import pyopenpose as op
        except ImportError:
            try:
                import pyopenpose as op
            except ImportError as exc:
                raise RuntimeError("OpenPose Python binding is not installed") from exc

        params = {
            "model_folder": model_folder,
            "model_pose": body_model,
            "net_resolution": net_resolution,
            "num_gpu_start": gpu_id,
        }
        self._op_wrapper = op.WrapperPython()
        self._op_wrapper.configure(params)
        self._op_wrapper.start()
        self._op_datum_class = op.Datum
        self._op_vector_datum_class = op.VectorDatum

    def infer(self, image: np.ndarray) -> OpenPoseResult:
        datum = self._op_datum_class()
        datum.cvInputData = image
        self._op_wrapper.emplaceAndPop(self._op_vector_datum_class([datum]))
        overlay = datum.cvOutputData if datum.cvOutputData is not None else image
        return OpenPoseResult(
            keypoints=self._extract_keypoints(datum.poseKeypoints),
            overlay=overlay,
        )

    def _extract_keypoints(self, pose_keypoints: Optional[np.ndarray]) -> List[DetectedKeypoint]:
        if pose_keypoints is None or len(pose_keypoints) == 0:
            return []
        person = self._select_person(pose_keypoints)
        if person is None:
            return []
        keypoints = []
        for name, index in keypoint_indices_for_side(self._side).items():
            if index >= len(person):
                continue
            x, y, confidence = person[index]
            if confidence < self._min_confidence:
                continue
            keypoints.append(
                DetectedKeypoint(
                    name=name,
                    x=float(x),
                    y=float(y),
                    confidence=float(confidence),
                )
            )
        return keypoints

    def _select_person(self, pose_keypoints: np.ndarray):
        side_indices = keypoint_indices_for_side(self._side).values()
        best_person = None
        best_area = -1.0
        for person in pose_keypoints:
            confident_points = [
                person[index]
                for index in side_indices
                if index < len(person) and person[index][2] >= self._min_confidence
            ]
            if len(confident_points) < 2:
                continue
            points = np.asarray(confident_points)
            width = float(points[:, 0].max() - points[:, 0].min())
            height = float(points[:, 1].max() - points[:, 1].min())
            area = width * height
            if area > best_area:
                best_area = area
                best_person = person
        return best_person
