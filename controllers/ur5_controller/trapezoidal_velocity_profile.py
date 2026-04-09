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
        self.time = 0.0
        self.ramp_time = self.calc_ramp_time()

    def ramp_up(self, joint_velocities: np.ndarray, normalized_joint_velocities: np.ndarray):
        if not self.max_speed_reached:
            # v = v0 + (a * t) but capped at a max linear speed
            self.current_speed = self.current_speed + (MotionConstants.LINEAR_ACCEL * (IntConstants.TIMESTEP / 1000.0))
            new_joint_velocities = normalized_joint_velocities * self.current_speed

            if self.current_speed >= MotionConstants.MAX_LINEAR_SPEED:
                self.max_speed_reached = True
        else:
            new_joint_velocities = joint_velocities
            
        return new_joint_velocities
    
    def calc_ramp_time(self):
        """
        The shortest time to go from 0 to max speed or from max speed to 0.
        """
        t = MotionConstants.MAX_LINEAR_SPEED / MotionConstants.LINEAR_ACCEL

        return t
    
    def dist_to_target(self, current_tf: np.ndarray, target_tf: np.ndarray):
        current_coords = current_tf[:3, 3]
        target_coords = target_tf[:3, 3]

        return np.linalg.norm(target_coords - current_coords)

    def min_time_to_target(self, dist_to_target: float):
        """
        Calculate the time required to reach the target if you move as fast as possible.
        """
        term_1 = (dist_to_target / 1000.0) / MotionConstants.MAX_LINEAR_SPEED
        term_2 = MotionConstants.MAX_LINEAR_SPEED / MotionConstants.LINEAR_ACCEL

        T = term_1 + term_2

        return T
    
    def ramp_down(self, joint_velocities: np.ndarray, normalized_joint_velocities: np.ndarray):
        if self.current_speed > 0:
            # v = v0 + (a * t)
            self.current_speed = self.current_speed - (MotionConstants.LINEAR_ACCEL * (IntConstants.TIMESTEP / 1000.0))
            new_joint_velocities = normalized_joint_velocities * self.current_speed
        else:
            new_joint_velocities = np.zeros(6)
            
        return new_joint_velocities
    
    def calc_min_braking_distance(self):
        return (self.current_speed ** 2) / (2 * MotionConstants.LINEAR_ACCEL)
    
    def trapezoid(
            self,
            current_tf: np.ndarray,
            target_tf: np.ndarray,
            joint_velocities: np.ndarray,
            normalized_joint_velocities: np.ndarray
    ):
        d = self.dist_to_target(current_tf, target_tf)
        T = self.min_time_to_target(d)
        mbt = self.calc_min_braking_distance()

        if T > mbt:
            new_joint_velocities = self.ramp_up(joint_velocities, normalized_joint_velocities)
        else:
            new_joint_velocities = self.ramp_down(joint_velocities, normalized_joint_velocities)

        return new_joint_velocities, self.current_speed