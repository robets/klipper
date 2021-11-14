from collections import deque
from common.consumer import consumer
from common.datatypes import SensorSample
from typing import Generator


@consumer
def window_average(n: int = 20) -> Generator[float, SensorSample, None]:
    # possible optimization: Add an accumulator to not need to sum the buffer every time.
    # Occasionally recalculate because floating point?
    n = max(1, n)
    buffer = deque(maxlen=n)
    next_value = yield 0
    buffer.append(next_value)
    while True:
        avg = sum(map(lambda s: s.value, buffer)) / (len(buffer) or 1)
        next_value = yield avg
        buffer.append(next_value)


@consumer
def exponential_moving_average(alpha: float = 0.5) -> Generator[float, SensorSample, None]:
    average = 0
    while True:
        current = yield average
        average = current.value * alpha + average * (1 - alpha)
