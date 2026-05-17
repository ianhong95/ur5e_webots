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
        self.step(self.TIMESTEP)

        self.ik_solver = IK_Solver()
        self.pid = PID_Controller()
        self.vel_profile = VelocityProfile()

        self.target_reached = False
        self.moving = False

        # We need to step ahead one timestep after initializing everything.
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

    def set_joint_velocities(self, velocities: np.ndarray):
        """Execution layer only."""
        for joint, motor in self.motors.items():
            motor.setPosition(float('inf'))
            motor.setVelocity(velocities[joint.idx])

    def set_joint_angles(self, joint_angles: list):
        for joint, motor in self.motors.items():
            motor.setPosition(joint_angles[joint.idx])

    def update_joint_angles(self):
        for joint, sensor in self.sensors.items():
            self.joint_angles[joint.idx] = sensor.getValue()

        return self.joint_angles
    
    # =================
    # FEEDBACK METHODS
    # =================
    

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

    def calculate_velocity_step(self, target_twist: np.ndarray):
        """
        Use inverse velocity kinematics to move the end-effector in a straight line.
        Follows a trapezoidal velocity profile.
        T_bd is the target TF expressed in the body frame.
        """

        self.update_joint_angles()

        # Update current pose T_sb and body Jacobian
        body_jacobian, self.T_sb = self.ik_solver.compute_body_jacobian(self.joint_angles)

        # Target velocities        
        joint_velocities, _ = self.ik_solver.compute_normalized_joint_velocities(target_twist, body_jacobian)

        return joint_velocities

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

    def actuate_gripper(self, position: float):
        """
        Blocking gripper control to ensure it's in the right position before
        proceeding to the next robot motion.

        0.0 = open
        1.0 = close
        """

        self.left_finger.setPosition(position)
        self.right_finger.setPosition(position)

        if gripper_error > Thresholds.GRIPPER_ERROR_THRESHOLD:
            gripper_error = abs(self.left_finger_sensor.getValue() - position)
        