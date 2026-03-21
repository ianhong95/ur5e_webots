from enum import Enum, IntEnum, StrEnum
from typing import NamedTuple, Final
from dataclasses import dataclass

import numpy as np


class IntConstants(IntEnum):
    TIMESTEP = 32   # 32ms per simulation step
    MAX_ITERATIONS = 100

@dataclass(frozen=True)
class FloatConstants:
    IK_ERROR_THRESHOLD: float = 1e-4
    THETA_THRESHOLD: float = 1e-3
    DAMPING_FACTOR: float = 1.0

@dataclass(frozen=True)
class MotionConstants:
    DEFAULT_LINEAR_VELOCITY: float = 200.0    # mm/s
    DEFAULT_ANGULAR_VELOCITY: float = 1.0   # rad/s
    MAX_ANGULAR_VELOCITY: float = 2.0   # rad/s
    MAX_LINEAR_VELOCITY: float = 1000.0   # mm/s

@dataclass(frozen=True)
class PhysicalParams:
    L0: float = 162.500
    L1: float = 425.000
    L2: float = 392.200
    L3: float = 99.700
    L4: float = 133.000
    L5: float = 99.600  # I don't know this exact value
    NUM_JOINTS = 6

# Alias
P = PhysicalParams

class PositionConstants():
    ZERO_TF: np.ndarray = np.array([
        [0, -1, 0, P.L1 + P.L2],
        [0, 0, 1, P.L4 + P.L5],
        [-1, 0, 0, P.L0 -P.L3],
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
        np.array([0, 0, P.L0])
    )
    
    SHOULDER_LIFT = JointInfo(
        1,
        'shoulder_lift_joint',
        'shoulder_lift_joint_sensor',
        np.array([0, 1, 0]),
        np.array([0, 0, P.L0])
    )
    
    ELBOW = JointInfo(
        2,
        'elbow_joint',
        'elbow_joint_sensor',
        np.array([0, 1, 0]),
        np.array([P.L1, 0, P.L0])
    )
    
    WRIST_1 = JointInfo(
        3,
        'wrist_1_joint',
        'wrist_1_joint_sensor',
        np.array([0, 1, 0]),
        np.array([P.L1 + P.L2, 0, P.L0])    
    )
    
    WRIST_2 = JointInfo(
        4,
        'wrist_2_joint',
        'wrist_2_joint_sensor',
        np.array([0, 0, -1]),
        np.array([P.L1 + P.L2, P.L4, P.L0])      
    )
    
    WRIST_3 = JointInfo(
        5,
        'wrist_3_joint',
        'wrist_3_joint_sensor',
        np.array([0, 1, 0]),
        np.array([P.L1 + P.L2, 0, P.L0-P.L3])    
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