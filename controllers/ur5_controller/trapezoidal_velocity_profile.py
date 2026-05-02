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
        self.time = 0.0
        self.t_cruise = 0.0
        self.T = 0.0

    def dist_to_target(self, current_tf: np.ndarray, target_tf: np.ndarray):
        current_coords = current_tf[:3, 3]
        target_coords = target_tf[:3, 3]

        norm =  np.linalg.norm(target_coords - current_coords)

        return norm
    
    def d_crit(self):
        """
        The distance required to accelerate from 0 to max speed.
        """
        return (M.MAX_LINEAR_SPEED ** 2) / M.MAX_LINEAR_ACCEL
    
    def calc_time_markers(self, T_sb: np.ndarray, T_sd: np.ndarray):
        self.d_total = self.dist_to_target(T_sb, T_sd)     #  convert to m

        # The distance required to accelerate from 0 to max speed.
        d_crit = (M.MAX_LINEAR_SPEED ** 2) / M.MAX_LINEAR_ACCEL

        if self.d_total < d_crit:
            # Triangle case
            self.v_peak = np.sqrt(self.d_total * M.MAX_LINEAR_ACCEL)
            self.t_ramp = self.v_peak / M.MAX_LINEAR_ACCEL
            self.t_cruise = 0.0
        else:
            self.v_peak = M.MAX_LINEAR_SPEED
            self.t_ramp = M.MAX_LINEAR_SPEED / M.MAX_LINEAR_ACCEL
            d_ramp_total = (M.MAX_LINEAR_SPEED**2) / M.MAX_LINEAR_ACCEL 
            d_cruise = self.d_total - d_ramp_total
            
            self.t_cruise = d_cruise / self.v_peak

        self.T = (2 * self.t_ramp) + self.t_cruise
        
    def calc_s(self, t: float, rot_error, trans_error):
        """
        Calculate s(t) and s_dot(t), the normalized path parameters.

        This is divided into 3 cases, one for each section of the trapezoid.

        Returns s and s_dot as scalars from 0 to 1.
        """

        # Case 1: Accelerate
        if t <= self.t_ramp:
            s_dot = M.MAX_LINEAR_ACCEL * t

            s = 0.5 * M.MAX_LINEAR_ACCEL * t ** 2

        # Case 2: Cruising
        elif t <= (self.t_ramp + self.t_cruise) and self.t_cruise > 0:
            s_dot = self.v_peak
            
            s = M.MAX_LINEAR_SPEED * t - ((M.MAX_LINEAR_SPEED**2) / (2 * M.MAX_LINEAR_ACCEL))

        # Case 3: Deceleration
        elif t <= self.T:
            # Derivative of s(t) from Modern Robotics
            s_dot = M.MAX_LINEAR_ACCEL * (self.T - t)
            
            # Ensure s_dot doesn't go negative due to float rounding
            s_dot = max(s_dot, 0.0)
            
            s_term_1 = 2 * M.MAX_LINEAR_ACCEL * M.MAX_LINEAR_SPEED * self.T
            s_term_2 = 2 * M.MAX_LINEAR_SPEED ** 2
            s_term_3 = (M.MAX_LINEAR_ACCEL**2) * (t - self.T)
            s_term_4 = 2 * M.MAX_LINEAR_ACCEL
            s = (s_term_1 - s_term_2 - s_term_3) / s_term_4
        
        # Case 4: Finished
        else:
            print(f'--- FINISHED t = {t} ---')
            s = 1
            s_dot = 0.0

        return s, s_dot