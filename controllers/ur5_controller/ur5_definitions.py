from enum import Enum, IntEnum, StrEnum
from typing import NamedTuple, Final
from dataclasses import dataclass

import numpy as np


class IntConstants(IntEnum):
    TIMESTEP = 32   # 32ms per simulation step
    MAX_ITERATIONS = 50

@dataclass(frozen=True)
class FloatConstants:
    IK_ERROR_THRESHOLD: float = 5e-2
    DAMPING_FACTOR: float = 1.0
    MAX_VELOCITY: float = 1.0   # rad/s * timestep

@dataclass(frozen=True)
class PhysicalParams:
    L0: float = 163.000
    L1: float = 425.000
    L2: float = 392.2500
    L3: float = 99.700
    L4: float = 133.000
    L5: float = 100.000
    NUM_JOINTS = 6

# Alias
P = PhysicalParams

class PositionConstants():
    ZERO_TF: np.ndarray = np.array([
        [0, -1, 0, P.L1 + P.L2],
        [0, 0, 1, P.L4 + P.L5],
        [-1, 0, 0, -P.L3],
        [0, 0, 0, 1]
    ])

    HOME_TF: np.ndarray = np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0]
    ])

class JointInfo(NamedTuple):
    index: int
    joint_name: str
    sensor_name: str
    axis: np.ndarray
    point: np.ndarray

# Built on top of JointInfo class
class Joint(Enum):
    SHOULDER_PAN = JointInfo(
        0,
        'shoulder_pan_joint',
        'shoulder_pan_joint_sensor',
        np.array([0, 0, 1]),
        np.array([0, 0, 0])
    )
    
    SHOULDER_LIFT = JointInfo(
        1,
        'shoulder_lift_joint',
        'shoulder_lift_joint_sensor',
        np.array([0, 1, 0]),
        np.array([0, 0, 0])
    )
    
    ELBOW = JointInfo(
        2,
        'elbow_joint',
        'elbow_joint_sensor',
        np.array([0, 1, 0]),
        np.array([P.L1, 0, 0])
    )
    
    WRIST_1 = JointInfo(
        3,
        'wrist_1_joint',
        'wrist_1_joint_sensor',
        np.array([0, 1, 0]),
        np.array([P.L1 + P.L2, 0, 0])    
    )
    
    WRIST_2 = JointInfo(
        4,
        'wrist_2_joint',
        'wrist_2_joint_sensor',
        np.array([0, 0, -1]),
        np.array([P.L1 + P.L2, P.L4, 0])      
    )
    
    WRIST_3 = JointInfo(
        5,
        'wrist_3_joint',
        'wrist_3_joint_sensor',
        np.array([0, 1, 0]),
        np.array([P.L1 + P.L2 + P.L3, 0, -(P.L4 + P.L5)])    
    )

    @property
    def idx(self):
        return self.value.index
    
    @property
    def name(self):
        return self.value.joint_name
    
    @property
    def sensor(self):
        return self.value.sensor_name
    
    @property
    def axis(self):
        return self.value.axis
    
    @property
    def point(self):
        return self.value.point
    
    @property
    def lin_velocity(self):
        return (-np.cross(self.axis, self.point))
    
    @property
    def screw_axis(self):
        return (np.concatenate([self.axis, self.lin_velocity]))