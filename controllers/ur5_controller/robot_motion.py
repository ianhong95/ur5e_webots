import numpy as np
from controllers.ur5_controller.ur5_controller import UR5Controller
from controllers.ur5_controller.kinematics import Kinematics

class RobotMotion(UR5Controller, Kinematics):
    """
    Math layer for computing targets based on coordinates and angles.

    Each method runs for one timestep.
    """

    def __init__(self):
        super().__init__()

    def move_x(self, x_target: float, start_pose: np.ndarray = None):
        target_coords = (x_target, 0, 0)
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rel_trans_xyz(target_coords, current_tf)

        self.moveL(target_tf, start_pose)

        return self
    
    def move_y(self, y_target: float):
        target_coords = (0, y_target, 0)

        self.update_joint_angles()

        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rel_trans_xyz(target_coords, current_tf)

        self.moveL(target_tf)

        return self
    
    def move_z(self, z_target: float):
        target_coords = (0, 0, z_target)
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rel_trans_xyz(target_coords, current_tf)

        self.moveL(target_tf)

        return self
    
    def rot_x(self, theta: float):
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_x(theta, current_tf)

        self.moveL(target_tf)

        return self
    
    def rot_y(self, theta: float):
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_y(theta, current_tf)

        self.moveL(target_tf)

        return self
    
    def rot_z(self, theta: float):
        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_z(theta, current_tf)

        self.moveL(target_tf)

        return self