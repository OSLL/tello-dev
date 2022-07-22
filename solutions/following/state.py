from enum import Enum, auto


class State(Enum):
    END = auto()
    READY = auto()
    START = auto()
    MOVE = auto()