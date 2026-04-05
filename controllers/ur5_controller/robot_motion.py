import numpy as np

from ur5_definitions import Joint, IntConstants, Thresholds, PhysicalParams, Tuning, MotionConstants
from ur5_controller import UR5Controller
from kinematics import Kinematics

class RobotMotion(UR5Controller, Kinematics):
    def __init__(self):
        super().__init__()

    def move_x(self, x_target: float):
        target_coords = (x_target, 0, 0)
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rel_trans_xyz(target_coords, current_tf)

        self.go_to_speed(target_tf)

        return self
    
    def move_y(self, y_target: float):
        target_coords = (0, y_target, 0)

        self.update_joint_angles()

        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rel_trans_xyz(target_coords, current_tf)

        self.go_to_speed(target_tf)

        return self
    
    def move_z(self, z_target: float):
        target_coords = (0, 0, z_target)
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rel_trans_xyz(target_coords, current_tf)

        self.go_to_speed(target_tf)

        return self
    
    def rot_x(self, theta: float):
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_x(theta, current_tf)

        self.go_to_speed(target_tf)

        return self
    
    def rot_y(self, theta: float):
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_y(theta, current_tf)

        self.go_to_speed(target_tf)

        return self
    
    def rot_z(self, theta: float):
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_z(theta, current_tf)

        self.go_to_speed(target_tf)

        return self
