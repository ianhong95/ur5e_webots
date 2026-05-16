import numpy as np

from controller import (Robot, Keyboard)

from controllers.ur5_controller.ur5_controller import UR5Controller
from controllers.ur5_controller.robot_motion import RobotMotion
from controllers.ur5_controller.ur5_definitions import (
    MotionConstants as M
)
from controllers.keyboard_controller.kb_controller_defs import KeyMap, MoveStates, Constants
from demo_scripts.test_positions import Positions

class KeyboardController(RobotMotion):
    def __init__(self):
        super().__init__()
        self.kb = Keyboard()
        self.kb.enable(self.controller.TIMESTEP)

        self.move_state = MoveStates.STOPPED
        self.speed = 0.0    # mm/s
        self.prev_keys = set()

    def get_keys(self):
        key = self.kb.getKey()

        pressed_keys = set()

        while key != -1:
            pressed_keys.add(key)
            key = self.kb.getKey()

        return pressed_keys
    
    def get_state(self, prev_keys: set):
        new_keys = self.get_keys()
        held_keys = new_keys & prev_keys
        released_keys = prev_keys - new_keys

        for key in new_keys:
            self.prev_keys.add(key)

        return new_keys, held_keys, released_keys
    
    def key_pressed(self, key: int):
        if self.move_state is MoveStates.STOPPED:
            self.update_joint_angles()
            _, T_sb = self.space_forward_kinematics(self.joint_angles)

        match key:
            case (KeyMap.FORWARD):
                self.move_x(Constants.MOVE_STEP, T_sb)
            case (KeyMap.BACKWARD):
                self.move_x(-Constants.MOVE_STEP, T_sb)
            case (KeyMap.LEFT):
                self.move_y(-Constants.MOVE_STEP, T_sb)
            case (KeyMap.RIGHT):
                self.move_y(Constants.MOVE_STEP, T_sb)
            case (KeyMap.UP):
                self.move_z(Constants.MOVE_STEP, T_sb)
            case (KeyMap.DOWN):
                self.move_z(-Constants.MOVE_STEP, T_sb)
            case (KeyMap.CLOSE):
                self.set_gripper(1.0)
            case (KeyMap.OPEN):
                self.set_gripper(0.0)

        self.move_state == MoveStates.MOVING

    def key_held(self, key: set):
        self.update_joint_angles()
        _, T_sb = self.space_forward_kinematics(self.joint_angles)

        match key:
            case (KeyMap.FORWARD):
                self.move_x(Constants.MOVE_STEP, T_sb)
            case (KeyMap.BACKWARD):
                self.move_x(-Constants.MOVE_STEP, T_sb)
            case (KeyMap.LEFT):
                self.move_y(-Constants.MOVE_STEP, T_sb)
            case (KeyMap.RIGHT):
                self.move_y(Constants.MOVE_STEP, T_sb)
            case (KeyMap.UP):
                self.move_z(Constants.MOVE_STEP, T_sb)
            case (KeyMap.DOWN):
                self.move_z(-Constants.MOVE_STEP, T_sb)
            case (KeyMap.CLOSE):
                self.set_gripper(1.0)
            case (KeyMap.OPEN):
                self.set_gripper(0.0)

    def key_released(self):
        self.move_state = MoveStates.STOPPED

    def go_to_home(self):
        self.controller.moveL(Positions.HOME)

        while self.step(self.controller.TIMESTEP) != -1:
            if not self.target_reached:
                self.controller.moveL(Positions.HOME)
            elif self.target_reached:
                print(f'Moved to home')