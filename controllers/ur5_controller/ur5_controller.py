import sys, os

os.environ['WEBOTS_ROBOT_NAME'] = 'UR5e'

from controller import Robot, DistanceSensor, Motor, PositionSensor
from math import pi
import numpy as np
import time

from ur5_definitions import Joint, IntConstants, FloatConstants, PhysicalParams
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
            [1.000, 0.000, 0.000, 200.000],
            [0.000, -1.000, 0.000, 133.000],
            [0.000, 0.000, -1.000, 200.000],
            [0.000, 0.000, 0.000, 1.000]
        ]),
        'TEST2': np.array([
            [1.000, 0.000, 0.000, 100.000],
            [0.000, -1.000, 0.000, 133.000],
            [0.000, 0.000, -1.000, 250.000],
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
            delta_theta, rot_error, trans_error = self.k.inv_kinematics(target_tf, target_joint_angles)

            if rot_error > FloatConstants.IK_ERROR_THRESHOLD or trans_error > FloatConstants.IK_ERROR_THRESHOLD:
                for idx, angle_increment in enumerate(delta_theta):
                    target_joint_angles[idx] += FloatConstants.DAMPING_FACTOR * angle_increment.item()
            else:
                break
        
        # Calculate linear distance between target and current pose
        linear_dist = abs(np.linalg.norm(target_tf[:3, 3] - initial_pose[:3, 3]))

        # Calculate number of timesteps required to move at the provided speed.
        mm_per_step = linear_speed * IntConstants.TIMESTEP / 1000

        num_steps_linear = np.clip(int(linear_dist / mm_per_step), 1, 200)

        # Extract rotation matrices
        R_initial = initial_pose[:3, :3]
        R_target = target_tf[:3, :3]
        
        # Calculate relative rotation angle
        R_rel = R_initial.T @ R_target
        acos_arg = np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)
        angular_dist = abs(np.arccos(acos_arg)) # Radian difference
        
        rad_per_step = angular_speed * IntConstants.TIMESTEP / 1000.0
        num_steps_angular = int(angular_dist / rad_per_step) if rad_per_step > 0 else 0

        # Put a cap on the number of steps
        num_steps = np.clip(max(num_steps_linear, num_steps_angular), 1, 200)
        
        # Calculate the angle steps required by each joint at each timestep
        angles_per_step = []
        for joint, motor in self.motors.items():
            angle_diff = target_joint_angles[joint.idx] - self.joint_angles[joint.idx]
            angles_per_step.append(angle_diff / num_steps)    # rad/step

        while self.step(self.TIMESTEP) != -1:
            self.target_reached = True

            for joint, motor in self.motors.items():
                angle_error = abs(target_joint_angles[joint.idx] - self.joint_angles[joint.idx])

                if angle_error > FloatConstants.IK_ERROR_THRESHOLD:
                    self.joint_angles[joint.idx] += angles_per_step[joint.idx]
                    motor.setPosition(self.joint_angles[joint.idx])
                    print(f'Target not reached. Error: {angle_error}')
                    self.target_reached = False
            
            if self.target_reached:
                print(f'Target reached!')
                return

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

                if joint_angle_error > FloatConstants.IK_ERROR_THRESHOLD:
                    print(f'Target not reached. Error: {joint_angle_error}')
                    self.target_reached = False
                    break
                else:
                    self.target_reached = True
            
            if self.target_reached:
                print(f'Target reached!')
                break

    def move_x(self, x_target: float):
        target_coords = (x_target, 0, 0)
        _, current_tf = self.k.body_forward_kinematics(self.joint_angles)

        target_tf = self.k.rel_trans_xyz(target_coords, current_tf)

        self.go_to_position(target_tf)

        return self
    
    def move_y(self, y_target: float):
        target_coords = (0, y_target, 0)
        self.update_joint_angles()
        self.step(self.TIMESTEP)
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
    robot.update_joint_angles()
    robot.k.body_forward_kinematics(robot.joint_angles)
    time.sleep(1)
    while True:
        robot.move_y(200)
        robot.move_y(-200)
        robot.rot_y(pi/4)
        robot.rot_y(-pi/4)

if __name__ == '__main__':
    main()