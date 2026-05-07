import sys, os

os.environ['WEBOTS_ROBOT_NAME'] = 'UR5e'

from controller import (
    Robot, DistanceSensor, Motor, PositionSensor, Keyboard
)
from math import pi
import numpy as np
import time
import multiprocessing as mp

from controllers.ur5_controller.ur5_definitions import Joint, Gripper, IntConstants, Thresholds
from controllers.ur5_controller.kinematics import Kinematics
from utilities.pid_error_plot import ErrorPlot
from controllers.ur5_controller.ik_solver_newton_raphson import IK_Solver
from controllers.ur5_controller.pid_helper import PID_Controller
from controllers.ur5_controller.trapezoidal_velocity_profile import VelocityProfile

class UR5Controller(Robot, Kinematics):

    TIMESTEP = IntConstants.TIMESTEP

    def __init__(self):
        super().__init__()
        print(f"Using Python: {sys.executable}")
        print("Connecting to Webots...")

        self._init_joints_and_sensors()
        self._init_gripper()
        self.ik_solver = IK_Solver()
        self.pid = PID_Controller()
        self.vel_profile = VelocityProfile()

        self.target_reached = False

        # We need to step ahead one timestep after initializing everything.
        self.step(self.TIMESTEP)
        self.update_joint_angles()
        self.step(self.TIMESTEP)

        self.parent_conn, child_conn = mp.Pipe()
        self.error_plot = ErrorPlot(child_conn)
        self.error_plot.start()

        time.sleep(1)

    def _list_devices(self):
        num_devices = self.getNumberOfDevices()
        print(f'There are {num_devices} devices.')

        for device in range(num_devices):
            device_obj = self.getDeviceByIndex(device)
            print(device_obj.name)

    def _init_joints_and_sensors(self):
        self.motors: dict[Joint, Motor] = {}
        self.sensors: dict[Joint, PositionSensor] = {}

        for j in Joint:
            self.motors[j] = self.getDevice(j.name)
            self.sensors[j] = self.getDevice(j.sensor)

        print(f'Joints and sensors initialized.')

        for sensor in self.sensors.values():
            sensor.enable(self.TIMESTEP)

        self.joint_angles = [0.00000] * len(Joint)

        print(f'Sensors enabled.')

    def reset_motor_speeds(self):
        for motor in self.motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)        
    
    # =================
    # FEEDBACK METHODS
    # =================
        
    def update_joint_angles(self):
        for joint, sensor in self.sensors.items():
            self.joint_angles[joint.idx] = sensor.getValue()

    def go_to_position(self, target_tf: np.ndarray, linear_speed: float = 200.0, angular_speed: float = 1.0):
        """
        Use inverse kinematics to move to a target TF.

        Linear speed is 100 mm/s by default.
        """
        self.update_joint_angles()
        self.step(self.TIMESTEP)

        target_joint_angles = self.joint_angles.copy()
        _, initial_pose = self.body_forward_kinematics(self.joint_angles)

        # Solve inverse kinematics numerically using Newton-Raphson
        for i in range(IntConstants.MAX_ITERATIONS):

            # Set the initial guess to the "target_joint_angles" which are initially the current joint angles
            delta_theta, rot_error, trans_error, twist_error_6D = self.inv_kinematics(target_tf, target_joint_angles)

            if rot_error > Thresholds.IK_ERROR_THRESHOLD or trans_error > Thresholds.IK_ERROR_THRESHOLD:
                for idx, angle_increment in enumerate(delta_theta):
                    target_joint_angles[idx] += Thresholds.DAMPING_FACTOR * angle_increment.item()
            else:
                break

        for joint, motor in self.motors.items():
            motor.setPosition(target_joint_angles[joint.idx])

        while self.step(self.TIMESTEP) != -1:
            self.target_reached = True
            self.update_joint_angles()
            for current, target in zip(self.joint_angles, target_joint_angles):
                error = target - current
                if (abs(error)) > Thresholds.THETA_THRESHOLD:
                    print(f'Target not reached yet. Error: {error}')
                    self.target_reached = False

            if self.target_reached:
                return
            
    def moveL(self, T_sd: np.ndarray, T_sb: np.ndarray = None):
        """
        Use inverse velocity kinematics to move the end-effector in a straight line.
        Follows a trapezoidal velocity profile.
        T_bd is the target TF in the body frame.
        """
        self.pid.reset()
        self.ik_solver.reset()
        self.reset_motor_speeds()
        self.vel_profile.reset()
        self.update_joint_angles()

        # Get starting pose and initial twist vector.
        if T_sb is None:
            _, T_sb = self.space_forward_kinematics(self.joint_angles)

        # Compute t_ramp_time, t_cruise, and T
        self.vel_profile.calc_time_markers(T_sb, T_sd)

        t = 0.0

        while self.step(self.TIMESTEP) != -1:
            t += (IntConstants.TIMESTEP / 1000.0)

            # Motion time won't always be a perfect multiple of the timestep
            if t > self.vel_profile.T:
                t = self.vel_profile.T

            self.update_joint_angles()

            # Update current pose T_sb and body Jacobian
            body_jacobian, T_sb = self.ik_solver.compute_body_jacobian(self.joint_angles)

            # Update errors based on updated pose
            rot_error, trans_error, twist_error_6D = self.ik_solver.compute_twist_errors(T_sb, T_sd)

            # Calculate s(t) and s_dot(t)
            s, s_dot = self.vel_profile.calc_s(t, rot_error, trans_error)

            # Update the unit twist direction vector.
            # Normalize the twist vector by the total linear distance.
            if trans_error > Thresholds.TRANS_ERROR_THRESHOLD: # Avoid division by zero at the very end
                unit_twist = twist_error_6D / trans_error
            else:
                unit_twist = twist_error_6D # Or zero if you're close enough            

            # Scale the unit twist vector by the linear speed to get the speed vector.
            scaled_twist = unit_twist * s_dot

            # Target velocities        
            joint_velocities, _ = self.ik_solver.compute_normalized_joint_velocities(scaled_twist, body_jacobian)

            # This is for plotting
            self.parent_conn.send((twist_error_6D, s_dot))

            # Send velocities to motors
            for joint, motor in self.motors.items():
                scalar_joint_velocity =  joint_velocities[joint.idx].reshape(())
                motor.setVelocity(scalar_joint_velocity)

            if t >= self.vel_profile.T:
                break

    def set_joint_angles(self, joint_angle_list: list[float]):
        """
        Sets an absolute joint angle in radians.
        """
        for joint, motor in self.motors.items():
            motor.setPosition(joint_angle_list[joint.idx])
            self.joint_angles[joint.idx] = joint_angle_list[joint.idx]

        while self.step(self.TIMESTEP) != -1:
            for i, sensor in enumerate(self.sensors.values()):
                current_angle = sensor.getValue()

                joint_angle_error = abs(current_angle - joint_angle_list[i])

                if joint_angle_error > Thresholds.IK_ERROR_THRESHOLD:
                    self.target_reached = False
                    break
                else:
                    self.target_reached = True
            
            if self.target_reached:
                print(f'Target reached!')
                break
        
    # ================
    # VELOCITY CONTROL
    # ================

    def set_joint_velocity(self, joint_velocity_list: list[float]):
        """
        Velocity control for all joints.

        First we have to set the motor position to infinity, then set the joint velocity.
        """

        for joint, motor in self.motors.items():
            motor.setPosition(float('inf'))
            motor.setVelocity(joint_velocity_list[joint.idx])

    # ================
    # GRIPPER CONTROL
    # ================

    def _init_gripper(self):
        self.left_finger = self.getDevice(Gripper.LEFT_FINGER.name)
        self.getDevice(Gripper.LEFT_FINGER.sensor).enable(self.TIMESTEP)
        self.left_finger_sensor = self.getDevice(Gripper.LEFT_FINGER.sensor)
        self.right_finger = self.getDevice(Gripper.RIGHT_FINGER.name)
        self.getDevice(Gripper.RIGHT_FINGER.sensor).enable(self.TIMESTEP)
        self.right_finger_sensor = self.getDevice(Gripper.RIGHT_FINGER.sensor)

    def set_gripper_continuous(self, position: float):
        """
        Non-blocking gripper control.

        0.0 = open
        1.0 = close
        """

        self.left_finger.setPosition(position)
        self.right_finger.setPosition(position)

    def set_gripper(self, position: float):
        """
        Blocking gripper control to ensure it's in the right position before
        proceeding to the next robot motion.

        0.0 = open
        1.0 = close
        """

        self.left_finger.setPosition(position)
        self.right_finger.setPosition(position)

        while self.step(self.TIMESTEP) != -1:
            gripper_error = abs(self.left_finger_sensor.getValue() - position)
            print(f'gripper_error: {gripper_error}')
            if gripper_error < Thresholds.GRIPPER_ERROR_THRESHOLD:
                break