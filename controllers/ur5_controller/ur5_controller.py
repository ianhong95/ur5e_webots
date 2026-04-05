import sys, os

os.environ['WEBOTS_ROBOT_NAME'] = 'UR5e'

from controller import Robot, DistanceSensor, Motor, PositionSensor
from math import pi
import numpy as np
import time
import multiprocessing as mp

from ur5_definitions import Joint, IntConstants, Thresholds, PhysicalParams, Tuning, MotionConstants
from kinematics import Kinematics
from utilities.pid_error_plot import ErrorPlot
from ik_solver_newton_raphson import IK_Solver
from pid_helper import PID_Controller
from trapezoidal_velocity_profile import VelocityProfile

class UR5Controller(Robot, Kinematics):

    TIMESTEP = IntConstants.TIMESTEP

    DEFAULT_POSITIONS = {
        'HORIZONTAL': [0.000, 0.000, 0.000, 0.000, 0.000, 0.000],
        'HOME': [0, -3*pi/4, pi/2, -pi/4, -pi/2, 0.000],
        'TEST': [0, -pi/2, pi/2, -pi/2, -pi/2, 0]
    }

    TEST_TARGETS = {
        'TEST': np.array([
            [1.000, 0.000, 0.000, 491.918],
            [0.000, -1.000, 0.000, 133.000],
            [0.000, 0.000, -1.000, 325.027],
            [0.000, 0.000, 0.000, 1.000]
        ]),
        'TEST2': np.array([
            [1.000, 0.000, 0.000, 491.918],
            [0.000, -1.000, 0.000, 133.000],
            [0.000, 0.000, -1.000, 225.027],
            [0.000, 0.000, 0.000, 1.000]
        ])
    }

    def __init__(self):
        super().__init__()
        print(f"Using Python: {sys.executable}")
        print("Connecting to Webots...")

        self._init_joints_and_sensors()
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
            
    def go_to_speed(self, target_tf: np.ndarray):
        """
        Use inverse velocity kinematics to move the end-effector in a straight line.
        Contains a PID loop and velocity ramp-up logic.
        """
        self.pid.reset()
        self.ik_solver.reset()
        self.reset_motor_speeds()
        self.vel_profile.reset()

        while self.step(self.TIMESTEP) != -1:
            self.update_joint_angles()
            target_joint_angles = self.joint_angles.copy()
            
            body_jacobian, T_sb = self.ik_solver.compute_body_jacobian(target_joint_angles)
            rot_error, trans_error, twist_error_6D = self.ik_solver.compute_twist_errors(T_sb, target_tf)

            pid_applied_twist_error = self.pid.compute_pid_error(twist_error_6D)
            self.parent_conn.send(pid_applied_twist_error)

            joint_velocities, normalized_joint_velocities = self.ik_solver.compute_normalized_joint_velocities(pid_applied_twist_error, body_jacobian)

            # Limit the ramped up speed
            joint_velocities = self.vel_profile.ramp_up(joint_velocities, normalized_joint_velocities)

            # TODO: This should probably be part of Newton-Raphson
            if abs(rot_error) > Thresholds.ROT_ERROR_THRESHOLD or abs(trans_error) > Thresholds.TRANS_ERROR_THRESHOLD:
                for joint, motor in self.motors.items():
                    scalar_joint_velocity =  joint_velocities[joint.idx].reshape(())
                    scalar_joint_velocity = np.clip(scalar_joint_velocity, -MotionConstants.MAX_JOINT_SPEED, MotionConstants.MAX_JOINT_SPEED)
                    motor.setVelocity(scalar_joint_velocity)
            else:
                for motor in self.motors.values():
                    motor.setVelocity(0.0)
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
                    # print(f'Target not reached. Error: {joint_angle_error}')
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