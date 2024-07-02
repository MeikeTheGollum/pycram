"""Module holding all enums of PyCRAM."""

from enum import Enum, auto


class Arms(Enum):
    """Enum for Arms."""
    LEFT = auto()
    RIGHT = auto()
    BOTH = auto()


class TaskStatus(Enum):
    """
    Enum for readable descriptions of a tasks' status.
    """
    CREATED = 0
    RUNNING = 1
    SUCCEEDED = 2
    FAILED = 3


class JointType(Enum):
    """
    Enum for readable joint types.
    """
    REVOLUTE = 0
    PRISMATIC = 1
    SPHERICAL = 2
    PLANAR = 3
    FIXED = 4


class Grasp(Enum):
    """
    Enum for Grasp orientations.
    """
    FRONT = 0
    LEFT = 1
    RIGHT = 2
    TOP = 3


class ObjectType(Enum):
    """
    Enum for Object types to easier identify different objects
    """
    METALMUG = auto()
    PRINGLES = auto()
    MILK = auto()
    SPOON = auto()
    BOWL = auto()
    BREAKFAST_CEREAL = auto()
    JEROEN_CUP = auto()
    ROBOT = auto()
    ENVIRONMENT = auto()
    GENERIC_OBJECT = auto()
    HUMAN = auto()


class State(Enum):
    """
    Enumeration which describes the result of a language expression.
    """
    SUCCEEDED = 1
    FAILED = 0
    RUNNING = 2
    INTERRUPTED = 3

class FilterConfig(Enum):
    """
    Declare existing filter methods.
    Currently supported: Butterworth
    """
    butterworth = 1

class ImageEnum(Enum):
    HI = 0
    TALK = 1
    DISH = 2
    DONE = 3
    DROP = 4
    HANDOVER = 5
    ORDER = 6
    PICKING = 7
    PLACING = 8
    REPEAT = 9
    SEARCH = 10
    WAVING = 11
    FOLLOWING = 12
    DRIVINGBACK = 13
    PUSHBUTTONS = 14
    FOLLOWSTOP = 15
    JREPEAT = 16
    SOFA = 17