from dataclasses import dataclass
from enum import Enum


class Direction(Enum):
    LEFT_UP = 0
    LEFT_DOWN = 1
    RIGHT_UP = 2
    RIGHT_DOWN = 3
    NONE = -1


@dataclass
class Position:
    x: float
    y: float
    z: float
