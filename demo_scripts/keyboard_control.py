import numpy as np

from controller import (Robot, Keyboard)

from controllers.ur5_controller.ur5_controller import UR5Controller
from controllers.ur5_controller.robot_motion import RobotMotion
from controllers.ur5_controller.ur5_definitions import (
    MotionConstants as M
)
from controllers.keyboard_controller.kb_controller_defs import KeyMap, MoveStates, Constants
from controllers.keyboard_controller.keyboard_controller import KeyboardController
from demo_scripts.test_positions import Positions


def run():
    kb = KeyboardController()

    kb.move_abs(Positions.HOME)
    kb.prime()

    while kb.controller.step(kb.controller.TIMESTEP) != -1:
        kb.update_state(kb.controller.TIMESTEP / 1000.0) 

        kb.update_kb_state()