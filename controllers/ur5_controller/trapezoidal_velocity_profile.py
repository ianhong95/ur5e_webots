import numpy as np
from ur5_definitions import Joint, IntConstants, Thresholds, PhysicalParams, Tuning, MotionConstants
from pid_helper import PID_Controller as pid

class VelocityProfile():
    """
    Trapezoidal velocity profile.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.current_speed = 0.0     # m/s
        self.max_speed_reached = False        

    def ramp_up(self, joint_velocities: np.ndarray, normalized_joint_velocities: np.ndarray):
        if not self.max_speed_reached:
            # v = v0 + (a * t) but capped at a max linear speed
            self.current_speed = min(MotionConstants.MAX_LINEAR_SPEED, self.current_speed + (MotionConstants.LINEAR_ACCEL * (IntConstants.TIMESTEP / 1000.0)))
            new_joint_velocities = normalized_joint_velocities * self.current_speed

            if self.current_speed >= MotionConstants.MAX_LINEAR_SPEED:
                self.max_speed_reached = True
        else:
            new_joint_velocities = joint_velocities
            
        return new_joint_velocities
        
    