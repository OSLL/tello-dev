from enum import Enum, auto


class State(Enum):
    START = auto()
    END = auto()
    READY = auto()
    MOVE = auto()
    ROTATE = auto()