import numpy as np

from ur5_definitions import Joint, IntConstants, Thresholds, PhysicalParams, Tuning, MotionConstants
from kinematics import Kinematics

class IK_Solver():
    def __init__(self):
        self.k = Kinematics()
        self.reset()

    def compute_body_jacobian(self, target_joint_angles: list[float]):
        exp, T_sb = self.k.body_forward_kinematics(target_joint_angles)
        current_jacobian = self.k.body_jacobian(exp)

        return current_jacobian, T_sb

    def compute_twist_errors(
            self,
            T_sb: np.ndarray,
            target_tf: np.ndarray,
    ):
        """
        Computes and processes the twist error for one iteration.
        """
        twist_error = self.k.compute_twist_error(T_sb, target_tf)

        twist_error_6D = self.k.se3_to_twist(twist_error, 'v')
        rot_error = np.linalg.norm(twist_error_6D[:3])
        trans_error = np.linalg.norm(twist_error_6D[3:6])

        return rot_error, trans_error, twist_error_6D

    def compute_normalized_joint_velocities(
            self,
            pid_applied_twist_error: np.ndarray,
            body_jacobian: np.ndarray
    ):
        self.normalized_twist = np.linalg.norm(pid_applied_twist_error)
        joint_velocities = np.linalg.pinv(body_jacobian) @ pid_applied_twist_error
        
        # The norm is the magnitude        
        joint_velocities_norm = np.linalg.norm(joint_velocities)

        # Normalize to isolate the 'direction' (as a unit vector) by dividing by magnitude
        # In this case, 'direction' is a normalized ratio of joint velocities relative to each other.        
        normalized_joint_velocities = joint_velocities / joint_velocities_norm

        return joint_velocities, normalized_joint_velocities
    
    def reset(self):
        self.normalized_twist = 0