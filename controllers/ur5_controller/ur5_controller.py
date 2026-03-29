import sys, os

os.environ['WEBOTS_ROBOT_NAME'] = 'UR5e'

from controller import Robot, DistanceSensor, Motor, PositionSensor
from math import pi
import numpy as np
import time

from ur5_definitions import Joint, IntConstants, Thresholds, PhysicalParams, Tuning
from kinematics import Kinematics


class UR5Controller(Robot):

    TIMESTEP = IntConstants.TIMESTEP

    DEFAULT_POSITIONS = {
        'HORIZONTAL': [0.000, 0.000, 0.000, 0.000, 0.000, 0.000],
        'HOME': [0, -3*pi/4, pi/2, -pi/4, -pi/2, 0.000],
        'TEST': [0, -pi/2, pi/2, -pi/2, -pi/2, 0]
        # 'TEST': [0, 0, 0, 0, 0, pi/2]
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
        self.k = Kinematics()
        self.target_reached = False

        # We need to step ahead one timestep after initializing everything.
        self.step(self.TIMESTEP)
        self.update_joint_angles()
        self.step(self.TIMESTEP)

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
    
    # =================
    # FEEDBACK METHODS
    # =================
        
    def update_joint_angles(self):
        for joint, sensor in self.sensors.items():
            self.joint_angles[joint.idx] = sensor.getValue()
        
        # Debug printing
        # joint_angles = [f'{angle:.5f}' for angle in self.joint_angles]

    def go_to_position(self, target_tf: np.ndarray, linear_speed: float = 200.0, angular_speed: float = 1.0):
        """
        Use inverse kinematics to move to a target TF.

        Linear speed is 100 mm/s by default.
        """
        self.update_joint_angles()
        self.step(self.TIMESTEP)

        target_joint_angles = self.joint_angles.copy()
        _, initial_pose = self.k.body_forward_kinematics(self.joint_angles)

        # Solve inverse kinematics numerically using Newton-Raphson
        for i in range(IntConstants.MAX_ITERATIONS):

            # Set the initial guess to the "target_joint_angles" which are initially the current joint angles
            delta_theta, rot_error, trans_error, twist_error_6D = self.k.inv_kinematics(target_tf, target_joint_angles)

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
        """
        for motor in self.motors.values():
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        integral_error = np.zeros(6)
        delta_t = self.TIMESTEP / 1000.0

        while self.step(self.TIMESTEP) != -1:
            self.update_joint_angles()

            target_joint_angles = self.joint_angles.copy()

            exp, T_sb = self.k.body_forward_kinematics(target_joint_angles)
            current_jacobian = self.k.body_jacobian(exp)
            twist_error = self.k.compute_twist_error(T_sb, target_tf)

            twist_error_6D = self.k.se3_to_twist(twist_error, 'v')
            rot_error = np.linalg.norm(twist_error_6D[:3])
            trans_error = np.linalg.norm(twist_error_6D[3:6])

            integral_error += twist_error_6D * delta_t

            new_twist_error = (twist_error_6D * Tuning.K_P) + (integral_error * Tuning.K_I)

            joint_velocities = np.linalg.pinv(current_jacobian) @ new_twist_error

            if rot_error > Thresholds.ROT_ERROR_THRESHOLD or trans_error > Thresholds.TRANS_ERROR_THRESHOLD:
                for joint, motor in self.motors.items():
                    scalar_joint_velocity = np.clip(joint_velocities[joint.idx].reshape(()), -1, 1)
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
                    print(f'Target not reached. Error: {joint_angle_error}')
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

    def move_x(self, x_target: float):
        target_coords = (x_target, 0, 0)
        _, current_tf = self.k.body_forward_kinematics(self.joint_angles)

        target_tf = self.k.rel_trans_xyz(target_coords, current_tf)

        # self.go_to_position(target_tf)
        self.go_to_speed(target_tf)

        return self
    
    def move_y(self, y_target: float):
        target_coords = (0, y_target, 0)

        self.update_joint_angles()

        _, current_tf = self.k.body_forward_kinematics(self.joint_angles)

        target_tf = self.k.rel_trans_xyz(target_coords, current_tf)

        self.go_to_position(target_tf)

        return self
    
    def move_z(self, z_target: float):
        target_coords = (0, 0, z_target)
        _, current_tf = self.k.body_forward_kinematics(self.joint_angles)

        target_tf = self.k.rel_trans_xyz(target_coords, current_tf)

        self.go_to_position(target_tf)

        return self
    
    def rot_x(self, theta: float):
        _, current_tf = self.k.body_forward_kinematics(self.joint_angles)

        target_tf = self.k.rot_x(theta, current_tf)

        self.go_to_position(target_tf)

        return self
    
    def rot_y(self, theta: float):
        _, current_tf = self.k.body_forward_kinematics(self.joint_angles)

        target_tf = self.k.rot_y(theta, current_tf)

        self.go_to_position(target_tf)

        return self
    
    def rot_z(self, theta: float):
        _, current_tf = self.k.body_forward_kinematics(self.joint_angles)

        target_tf = self.k.rot_z(theta, current_tf)

        self.go_to_position(target_tf)

        return self


def main():
    robot = UR5Controller()

    robot.set_joint_angles(robot.DEFAULT_POSITIONS['HOME'])
    robot.go_to_position(robot.TEST_TARGETS['TEST'])
    _, fk = robot.k.body_forward_kinematics(robot.joint_angles)
    # print(f'fk: {fk}')
    # print(f'joint_angles: {robot.joint_angles}')    
    
    while True:
        robot.move_x(-150)
        # robot.move_y(200)
        # robot.move_x(150)
        # robot.move_y(-400)
        # robot.move_x(-150)
        # robot.move_z(150)
        # robot.move_y(200)
        # robot.move_z(-150)
        # robot.rot_y(pi/4)
        # robot.rot_y(-pi/4)
        robot.move_x(150)

if __name__ == '__main__':
    main()