from controller import Robot, DistanceSensor, Motor, PositionSensor
import sys
from math import pi
import numpy as np
import time

from ur5_definitions import Joint, IntConstants, LimitConstants
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
            [1.000, 0.000, 0.000, 486.910],
            [0.000, -1.000, 0.000, 109.150],
            [0.000, 0.000, -1.000, 324.700],
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
        joint_angles = [f'{angle:.5f}' for angle in self.joint_angles]

    def go_to_position(self, target_tf: np.ndarray):
        """
        Use inverse kinematics to move to a target TF.
        """
        solution_converged = False

        while not solution_converged and self.step(self.TIMESTEP) != -1:
            # while self.step(self.TIMESTEP) != -1:
            angles_increment, rot_error, trans_error = self.k.inv_kinematics(target_tf, self.joint_angles)
            print(f'rot_error: {rot_error}')
            print(f'trans_error: {trans_error}')
            if rot_error > LimitConstants.IK_ERROR_THRESHOLD or trans_error > LimitConstants.IK_ERROR_THRESHOLD:
                for angle_increment, (joint, motor) in zip(angles_increment, self.motors.items()):
                    self.joint_angles[joint.idx] += angle_increment.item()
                    motor.setPosition(self.joint_angles[joint.idx])
                print(f'angles increment: {angles_increment}')
                print(f'current joint angles: {self.joint_angles}')

            else:
                solution_converged = True
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

                if joint_angle_error > LimitConstants.IK_ERROR_THRESHOLD:
                    print(f'Target not reached. Error: {joint_angle_error}')
                    self.target_reached = False
                    break
                else:
                    self.target_reached = True
            
            if self.target_reached:
                print(f'Target reached!')
                break

def main():
    robot = UR5Controller()

    # while (robot.step(robot.TIMESTEP) != -1):
    robot.set_joint_angles(robot.DEFAULT_POSITIONS['HOME'])
    # time.sleep(2)
    # robot.set_joint_angles(robot.DEFAULT_POSITIONS['HORIZONTAL'])
    # robot.update_joint_angles()
    if 'nan' not in robot.joint_angles:
        f = robot.k.body_forward_kinematics(robot.joint_angles)
    robot.go_to_position(robot.TEST_TARGETS['TEST'])

if __name__ == '__main__':
    main()