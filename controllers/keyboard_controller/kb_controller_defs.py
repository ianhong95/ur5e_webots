class KeyMap:
    FORWARD = ord('W')
    BACKWARD = ord('S')
    LEFT = ord('A')
    RIGHT = ord('D')
    UP = ord('R')
    DOWN = ord('F')
    CLOSE = ord('J')
    OPEN = ord('K')
    HOME = ord('H')

class MoveStates:
    STOPPED = 0
    ACCELERATING = 1
    CRUISING = 2
    DECELERATING = 3
    MOVING = 4

class Constants:
    MOVE_STEP = 50   # mm