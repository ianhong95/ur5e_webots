import numpy as np
from controllers.ur5_controller.ur5_controller import UR5Controller
from controllers.ur5_controller.kinematics import Kinematics
from controllers.ur5_controller.trapezoidal_velocity_profile import VelocityProfile
from controllers.ur5_controller.ur5_definitions import (
    Joint, Gripper, IntConstants, Thresholds
)
from controllers.ur5_controller.robot_commands import *

from collections import deque

class RobotMotion():
    """
    Math layer for computing targets based on coordinates and angles.

    Each method runs for one timestep.
    """

    def __init__(self):
        self.controller = UR5Controller()
        self.vel_profile = VelocityProfile()
        self.queue = deque()

        self.target_gripper_position = None
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
        """
        Computes target transformation matrix based on target Cartesian coordinates
        and computes time markers.
        """
        self.controller.update_joint_angles()
        _, current_tf = self.controller.body_forward_kinematics(self.controller.joint_angles) # T_sb
        target_tf = self.controller.rel_trans_xyz(target_coords, current_tf)   # T_sd

        _, _, T = self.vel_profile.calc_time_markers(current_tf, target_tf)

        return target_tf
    
    def rot_x(self, theta: float):
        """TODO: Reimplement or remove this"""
        self.is_moving = True

        _, current_tf = self.body_forward_kinematics(self.joint_angles)
        target_tf = self.rot_x(theta, current_tf)
        self.moveL(target_tf)

        return self
    
    def rot_y(self, theta: float):
        """TODO: Reimplement or remove this"""
        self.is_moving = True

        _, current_tf = self.body_forward_kinematics(self.joint_angles)
        target_tf = self.rot_y(theta, current_tf)
        self.moveL(target_tf)

        return self
    
    def rot_z(self, theta: float):
        """TODO: Reimplement or remove this"""
        self.is_moving = True

        _, current_tf = self.body_forward_kinematics(self.joint_angles)
        target_tf = self.rot_z(theta, current_tf)
        self.moveL(target_tf)

        return self
    
    def move_rel(self, x=0.0, y=0.0, z=0.0):
        """Queues a relative Cartesian translation step (inputs in mm)."""
        self.controller.update_joint_angles()
        self.queue.append(MoveRelCmd(x, y, z))

        return self
    
    def move_abs(self, target_tf: np.ndarray):
        """Queues a step to move to an absolute target."""
        self.controller.update_joint_angles()
        self.queue.append(MoveAbsCommand(target_tf))

        return self
    
    def set_gripper(self, position: float):
        self.queue.append(GripperCmd(position))
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
        
        # Start by getting the current joint angles
        self.controller.update_joint_angles()

        if not self.is_moving and self.queue:
            # Removes and returns an element from the left end
            command = self.queue.popleft()
            
            # Determine current position
            _, T_sb_start = self.controller.space_forward_kinematics(self.controller.joint_angles)

            # RUN the queued function
            command.execute(self, T_sb_start)

            self.t_elapsed = 0.0
            self.compute_vel_path(T_sb_start, self.T_sd)
            self.controller.reset_motor_speeds()

            self.is_moving = True

        if self.is_moving:
            self.t_elapsed += dt
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

            if self.target_gripper_position:
                gripper_error = abs(self.controller.left_finger_sensor.getValue() - self.target_gripper_position)
            else:
                gripper_error = 0.0

            if (self.t_elapsed >= self.vel_profile.T) and (gripper_error < Thresholds.GRIPPER_ERROR_THRESHOLD):
                self.reset()
                print(f'Motion complete!')

    # =========
    # UTILITIES
    # =========

    def add_to_queue(self, target_tf):
        self.queue.append(target_tf)

    def prime(self):
        """
        Locks the initial motion parameters BEFORE the simulation steps.
        Call this in your main script right after queuing your first motions.

        If the robot is not primed, it will fall slightly due to gravity at the first timestep
        before any calculations are done, which results in incorrect kinematics solutions
        such as elbow-down instead of elbow-up.
        """
        if not self.queue:
            return

        # Sync the sensors instantly while the arm is still perfectly frozen
        self.controller.update_joint_angles()
        
        # Capture the perfect zero position before it sags due to gravity
        _, T_sb_start = self.controller.space_forward_kinematics(self.controller.joint_angles)

        # Pop the first movement and resolve its target matrix
        command = self.queue.popleft()
        command.execute(self, T_sb_start)

        # Initialize path and profile
        self.compute_vel_path(T_sb_start, self.T_sd)
        self.controller.reset_motor_speeds()
        
        self.target_reached = False
        self.is_moving = True
