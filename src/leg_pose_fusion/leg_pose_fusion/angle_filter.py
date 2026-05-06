from dataclasses import dataclass
from typing import Optional


@dataclass
class LowPassFilter:
    cutoff_hz: float
    value: Optional[float] = None

    def update(self, sample: float, dt_s: float) -> float:
        if self.value is None or dt_s <= 0.0 or self.cutoff_hz <= 0.0:
            self.value = sample
            return sample
        rc = 1.0 / (2.0 * 3.141592653589793 * self.cutoff_hz)
        alpha = dt_s / (rc + dt_s)
        self.value = self.value + alpha * (sample - self.value)
        return self.value

    def reset(self) -> None:
        self.value = None


class AngleFilterSet:
    def __init__(self, cutoff_hz: float) -> None:
        self.hip = LowPassFilter(cutoff_hz)
        self.knee = LowPassFilter(cutoff_hz)
        self.ankle_dorsi = LowPassFilter(cutoff_hz)

    def reset(self) -> None:
        self.hip.reset()
        self.knee.reset()
        self.ankle_dorsi.reset()
