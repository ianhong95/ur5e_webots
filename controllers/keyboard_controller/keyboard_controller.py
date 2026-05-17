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
        self.prev_key = -1
        self.held_key = -1

    def get_keys(self):
        pressed_key = self.kb.getKey()

        return pressed_key
    
    def update_kb_state(self):
        new_key = self.get_keys()
        
        # Key pressed for the first time
        if new_key != -1 and self.prev_key == -1 and self.held_key == -1:
            print(f'Key pressed: {new_key}')
            self.prev_key = new_key
            self.key_pressed(new_key)
            return

        # Key pressed is same as previously pressed key (held)
        elif new_key != -1 and new_key == self.prev_key:
            self.held_key = new_key
            print(f'Holding key: {self.held_key}')
            self.key_held(new_key)
            return

        # A key is pressed but is different from previous key
        elif new_key != -1 and new_key != self.prev_key:
            print(f'New key pressed: {new_key}')
            self.key_released()
            self.prev_key = new_key
            self.key_pressed(new_key)
            return

        # Key is released
        elif new_key == -1 and self.prev_key != -1:
            print(f'Key released: {self.prev_key}')
            self.key_released()
            return

        # No key is pressed
        elif new_key == -1 and self.prev_key == -1:
            return
    
    def key_pressed(self, key: int):
        # if self.move_state is MoveStates.STOPPED:
        if self.is_moving:
            self.controller.update_joint_angles()
            _, T_sb = self.controller.space_forward_kinematics(self.controller.joint_angles)

        match key:
            case (KeyMap.FORWARD):
                self.move_rel(x=10.0)
            case (KeyMap.BACKWARD):
                self.move_rel(x=-10.0)
            case (KeyMap.LEFT):
                self.move_rel(y=10.0)
            case (KeyMap.RIGHT):
                self.move_rel(y=-10.0)
            case (KeyMap.UP):
                self.move_rel(z=10.0)
            case (KeyMap.DOWN):
                self.move_rel(z=-10.0)
            case (KeyMap.CLOSE):
                self.set_gripper(0.8)
            case (KeyMap.OPEN):
                self.set_gripper(0.2)
            case (KeyMap.HOME):
                self.move_abs(Positions.HOME)

        self.move_state == MoveStates.MOVING

    def key_held(self, key):
        self.controller.update_joint_angles()
        _, T_sb = self.controller.space_forward_kinematics(self.controller.joint_angles)

        match key:
            case (KeyMap.FORWARD):
                self.move_rel(x=10.0)
            case (KeyMap.BACKWARD):
                self.move_rel(x=-10.0)
            case (KeyMap.LEFT):
                self.move_rel(y=10.0)
            case (KeyMap.RIGHT):
                self.move_rel(y=-10.0)
            case (KeyMap.UP):
                self.move_rel(z=10.0)
            case (KeyMap.DOWN):
                self.move_rel(z=-10.0)
            case (KeyMap.CLOSE):
                self.set_gripper(0.8)
            case (KeyMap.OPEN):
                self.set_gripper(0.2)

    def key_released(self):
        self.is_moving = False
        self.held_key = -1
        self.prev_key = -1
        self.reset()
        self.queue.clear()
        self.controller.reset_motor_speeds()