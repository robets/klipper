from dataclasses import dataclass
from typing import NewType


PrintTime = NewType('PrintTime', float)
McuTime = NewType('McuTime', float)


@dataclass
class SensorSample:
    value: float
    time: PrintTime
