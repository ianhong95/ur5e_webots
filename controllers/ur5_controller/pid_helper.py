from kinematics import Kinematics
from ur5_definitions import Thresholds, IntConstants, PhysicalParams, Tuning, MotionConstants

import numpy as np

class PID_Controller():
    def __init__(self):
        self.reset()

    def compute_pid_error(
            self,
            twist_error_6D,
    ):
        self.integral_error += twist_error_6D * IntConstants.TIMESTEP / 1000.0
        self.integral_error = np.clip(self.integral_error, -0.5, 0.5)

        self.derivative_error = (twist_error_6D - self.previous_error) / (IntConstants.TIMESTEP / 1000.0)
        self.previous_error = twist_error_6D

        pid_applied_twist_error = (twist_error_6D * Tuning.K_P) + (self.integral_error * Tuning.K_I) + (self.derivative_error * Tuning.K_D)

        return pid_applied_twist_error

    def reset(self):
        self.integral_error = np.zeros(6)
        self.previous_error = np.zeros(6)