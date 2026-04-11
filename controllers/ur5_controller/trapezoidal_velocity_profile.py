import numpy as np
from ur5_definitions import Joint, IntConstants, Thresholds, PhysicalParams, Tuning
from ur5_definitions import MotionConstants as M
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
        self.t_ramp_up = 0.0
        self.t_cruise = 0.0
        self.T = 0.0

    def ramp_up(self):
        if not self.max_speed_reached:
            # v = v0 + (a * t) but capped at a max linear speed
            self.current_speed = self.current_speed + (M.LINEAR_ACCEL * (IntConstants.TIMESTEP / 1000.0))
    
    def calc_ramp_time(self):
        """
        The shortest time to go from 0 to max speed or from max speed to 0.
        """
        t = M.MAX_LINEAR_SPEED / M.MAX_LINEAR_ACCEL

        return t
    
    def dist_to_target(self, current_tf: np.ndarray, target_tf: np.ndarray):
        current_coords = current_tf[:3, 3]
        target_coords = target_tf[:3, 3]

        return np.linalg.norm(target_coords - current_coords)

    def min_time_to_target(self, dist_to_target: float):
        """
        Calculate the time required to reach the target if you move as fast as possible.
        """
        term_1 = (dist_to_target / 1000.0) / M.MAX_LINEAR_SPEED
        term_2 = M.MAX_LINEAR_SPEED / M.MAX_LINEAR_ACCEL

        T = term_1 + term_2

        return T
    
    def ramp_down(self):
        if self.current_speed > 0:
            # v = v0 + (a * t)
            self.current_speed = self.current_speed - (M.MAX_LINEAR_ACCEL * (IntConstants.TIMESTEP / 1000.0))
    
    def calc_min_braking_distance(self):
        return (self.current_speed ** 2) / (2 * M.MAX_LINEAR_ACCEL)
    
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
    
    def calc_lin_speed(self, current_tf: np.ndarray, target_tf: np.ndarray):
        d = self.dist_to_target(current_tf, target_tf)
        T = self.min_time_to_target(d)
        mbt = self.calc_min_braking_distance()

        if T > mbt:
            self.ramp_up()
        else:
            self.ramp_down()

        return self.current_speed
    
    def calc_time_markers(self):
        self.t_ramp_up = M.MAX_LINEAR_SPEED / M.MAX_LINEAR_ACCEL
        self.T = (M.MAX_LINEAR_ACCEL + (M.MAX_LINEAR_SPEED ** 2)) / (M.MAX_LINEAR_ACCEL * M.MAX_LINEAR_SPEED)
        self.t_cruise = self.T - 2 * self.t_ramp_up
    
    def calc_s(self, t: int):
        """
        Calculate s(t) and s_dot(t), the normalized path parameters.

        This is divided into 3 cases, one for each section of the trapezoid.

        Returns s and s_dot as scalars from 0 to 1.
        """

        # dist_remaining = self.dist_to_target

        # Case 1
        if t >= 0 and t <= self.t_ramp_up:
            s = 0.5 * M.MAX_LINEAR_ACCEL * t ** 2
            s_dot = M.MAX_LINEAR_ACCEL * t
        elif t > self.t_ramp_up and t <= (self.T - self.t_ramp_up):
            s = M.MAX_LINEAR_SPEED * t - ((M.MAX_LINEAR_SPEED ** 2) / (2 * M.MAX_LINEAR_ACCEL))
            s_dot = M.MAX_LINEAR_SPEED
        elif t > (self.T - self.t_ramp_up) and t <= self.T:
            # Split up terms for readability
            a = 2 * M.MAX_LINEAR_ACCEL * M.MAX_LINEAR_SPEED * self.T
            b = 2 * M.MAX_LINEAR_SPEED ** 2
            c = (M.MAX_LINEAR_ACCEL ** 2) * (t - self.T) ** 2
            d = 2 * M.MAX_LINEAR_ACCEL
            s = (a - b - c) / d

            s_dot = -M.MAX_LINEAR_ACCEL * (t - self.T)
        else:
            s = 0.0
            s_dot = 0.0
            print(f'Invalid normalized path case.')

        return s, s_dot


