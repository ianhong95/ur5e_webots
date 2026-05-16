import numpy as np
from controllers.ur5_controller.ur5_controller import UR5Controller
from controllers.ur5_controller.kinematics import Kinematics
from controllers.ur5_controller.trapezoidal_velocity_profile import VelocityProfile
from controllers.ur5_controller.ur5_definitions import (
    Joint, Gripper, IntConstants, Thresholds
)

class RobotMotion():
    """
    Math layer for computing targets based on coordinates and angles.

    Each method runs for one timestep.
    """

    def __init__(self):
        self.controller = UR5Controller()
        self.vel_profile = VelocityProfile()

        self.reset()

    # =======
    # MOTIONS
    # =======

    def reset(self):
        self.is_moving = False
        self.target_reached = False
        self.t_elapsed = 0.0

    def moveL(self, T_sd: np.ndarray):
        """
        Called ONCE to initialize a linear motion.
        Does NOT contain a loop.
        """
        
        # Get the starting pose. This does not change.
        self.controller.update_joint_angles()
        _, self.T_sb_start = self.controller.space_forward_kinematics(self.controller.joint_angles)
        
        self.T_sd = T_sd

        # 2. Reset the Motion Timer
        self.t_elapsed = 0.0

        # 3. Calculate Velocity Profile Markers
        # This determines T, t_ramp, and max_velocity based on distance
        self.compute_vel_path(self.T_sb_start, self.T_sd)

        # 4. Reset helper states
        self.controller.reset_motor_speeds()
        self.target_reached = False
        
        # 5. Flip the switch to let the 'update' method take over
        self.is_moving = True
        print(f"Motion initialized. Total duration: {self.vel_profile.T:.2f}s")

    def compute_target(self, target_coords:tuple):
        self.controller.update_joint_angles()
        _, current_tf = self.controller.body_forward_kinematics(self.controller.joint_angles) # T_sb
        target_tf = self.controller.rel_trans_xyz(target_coords, current_tf)   # T_sd

        _, _, T = self.vel_profile.calc_time_markers(current_tf, target_tf)

        return target_tf

    def move_x(self, x_target: float):
        self.is_moving = True
    
        target_coords = (x_target, 0, 0)
        target_tf = self.compute_target(target_coords)
        self.moveL(target_tf)

        return self
    
    def move_y(self, y_target: float):
        self.is_moving = True

        target_coords = (0, y_target, 0)
        target_tf = self.compute_target(target_coords)
        self.moveL(target_tf)

        return self
    
    def move_z(self, z_target: float):
        self.is_moving = True

        target_coords = (0, 0, z_target)
        target_tf = self.compute_target(target_coords)
        self.moveL(target_tf)

        return self
    
    def rot_x(self, theta: float):
        self.is_moving = True

        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_x(theta, current_tf)

        self.moveL(target_tf)

        return self
    
    def rot_y(self, theta: float):
        self.is_moving = True

        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_y(theta, current_tf)

        self.moveL(target_tf)

        return self
    
    def rot_z(self, theta: float):
        self.is_moving = True

        _, current_tf = self.body_forward_kinematics(self.joint_angles)

        target_tf = self.rot_z(theta, current_tf)

        self.moveL(target_tf)

        return self
    
    # ==========
    # KINEMATICS
    # ==========

    def compute_vel_path(self, T_sb: np.ndarray, T_sd: np.ndarray):
        self.vel_profile.reset()
        self.vel_profile.calc_time_markers(T_sb, T_sd)
        self.t_elapsed = 0.0

    def update_state(self, dt: int):
        """
        The heartbeat of the simulation loop to update the robot's state every timestep.

        :param dt: Time increment in ms.
        """
        if not self.is_moving:
            return
        
        self.t_elapsed += dt
        self.controller.update_joint_angles()
        _, self.T_sb = self.controller.space_forward_kinematics(self.controller.joint_angles)

        s, s_dot = self.vel_profile.calc_s(self.t_elapsed)

        # Update errors based on updated pose
        rot_error, trans_error, twist_error_6D = self.controller.ik_solver.compute_twist_errors(self.T_sb, self.T_sd)        

        # Update the unit twist direction vector.
        # Normalize the twist vector by the total linear distance.
        if trans_error > Thresholds.TRANS_ERROR_THRESHOLD: # Avoid division by zero at the very end
            unit_twist = twist_error_6D / trans_error
        else:
            unit_twist = twist_error_6D # Or zero if you're close enough

        # Scale the unit twist vector by the linear speed to get the speed vector.
        scaled_twist = unit_twist * s_dot

        joint_velocities = self.controller.calculate_velocity_step(scaled_twist)
        self.controller.set_joint_velocities(joint_velocities)

        # gripper_error = abs(self.controller.left_finger_sensor.getValue() - position)
        gripper_error = 0.0

        position = 0

        if (self.t_elapsed >= self.vel_profile.T) and (gripper_error < Thresholds.GRIPPER_ERROR_THRESHOLD):
            self.reset()


