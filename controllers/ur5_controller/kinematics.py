from math import sin, cos, tan, atan2, pi, acos
import json

import numpy as np

from ur5_definitions import Joint, IntConstants, FloatConstants, PhysicalParams, PositionConstants


class Kinematics():
    def __init__(self):
        self.BODY_SCREWS = self.space_to_body_screw()
        # exp, T = self.body_forward_kinematics([0, 0, 0, 0, 0, 0])
        # self.body_jacobian([0, 0, 0, 0, 0, 0], T, exp)

    # =============================
    # GENERAL COMPUTATION UTILITIES
    # =============================

    def unskew(self, skew_symmetric_matrix: np.ndarray):
        """
        Unskews a 3x3 skew-symmetric matrix by extracting the x, y, and z components.
        """
        return np.array([
            skew_symmetric_matrix[2, 1],
            skew_symmetric_matrix[0, 2],
            skew_symmetric_matrix[1, 0]
        ])
    
    def se3_to_twist(self, se3_matrix: np.ndarray, shape: str = 'v'):
        """
        Turns an SE(3) matrix into a 6x1 vector.
        """
        rotation = self.unskew(se3_matrix[:3, :3])
        linear = se3_matrix[:3, 3]

        if shape.lower() == 'h':
            return np.concatenate((rotation, linear)).reshape(1, 6)
        elif shape.lower() == 'v':
            return np.concatenate((rotation, linear)).reshape(6, 1)

    def space_to_body_screw(self):
        """
        Returns a list of 6x1 screw axes in the body (end-effector) frame.
        """
        M = PositionConstants.ZERO_TF

        # 4x4 matrix
        M_inv = np.linalg.inv(M)

        body_screws = []
        for joint in Joint:
            body_screws.append(self.adjoint_map(M_inv[:3, :3], M_inv[:3, 3]) @ joint.screw_axis)

        return body_screws

    def screw_vec_to_se3(self, screw_axis: np.ndarray):
        """
        Takes a 6x1 screw axis vector S and converts it to an se(3) matrix [S].
        """
        return np.array([
            [0, -screw_axis[2], screw_axis[1], screw_axis[3]],
            [screw_axis[2], 0, -screw_axis[0], screw_axis[4]],
            [-screw_axis[1], screw_axis[0], 0, screw_axis[5]],
            [0, 0, 0, 0]
        ])
    
    def skew_symmetric(self, vector: np.ndarray):
        """
        Takes the x, y, and z components of a vector and converts it into a
        3x3 skew-symmetric matrix.
        """
        wx = vector[0]
        wy = vector[1]
        wz = vector[2]

        return np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ])

    def compute_rodrigues_rot(self, omega_vector: np.ndarray, theta: float):
        """
        Computes the rotation matrix using Rodrigues' formula.
        """
        skew_symmetric_omega = self.skew_symmetric(omega_vector)
        R = np.eye(3) + sin(theta) * skew_symmetric_omega + (1 - cos(theta)) * (skew_symmetric_omega @ skew_symmetric_omega)

        return R
    
    def compute_p_vector(self, omega_vector: np.ndarray, theta: float, lin_velocity: np.ndarray):
        """
        Computes the p vector.
        """
        skew_symmetric_omega = self.skew_symmetric(omega_vector)
        p = (np.eye(3) * theta + (1 - cos(theta)) * skew_symmetric_omega + (theta - sin(theta)) * (skew_symmetric_omega @ skew_symmetric_omega)) @ lin_velocity.reshape(3, 1)

        return p
    
    def compute_matrix_exp(self, omega_vector: np.ndarray, theta: float, lin_velocity: np.ndarray):
        """
        Computes the matrix exponential.
        """
        R = self.compute_rodrigues_rot(omega_vector, theta)
        p = self.compute_p_vector(omega_vector, theta, lin_velocity)

        exp_mat = np.eye(4)
        exp_mat[:3, :3] = R             # Slice top left 3x3
        exp_mat[:3, 3] = p.flatten()    # Slice first 3 rows at column 4 (index 3)

        return exp_mat
    
    def adjoint_map(self, R: np.ndarray, p: np.ndarray):
        """
        Builds the 6x6 adjoint map matrix using the 3x3 rotation matrix R and
        3x1 translation vector p.
        """
        p_skew_symmetric = self.skew_symmetric(p)

        ad_map = np.eye(6)

        ad_map[:3, :3] = R
        ad_map[3:,:3] = p_skew_symmetric @ R
        ad_map[3:, 3:] = R

        return ad_map
    
    def adjoint_transform(self, R: np.ndarray, p: np.ndarray, v: np.ndarray, omega: np.ndarray):
        """
        Applies the adjoint map to a screw axis without building a 6x6 matrix.

        Returns a 1x6 matrix.
        """
        rotated_v = R @ v
        rotated_w = R @ omega

        linear_component = np.cross(p, rotated_w) + rotated_v

        transformed_screw = np.hstack((rotated_w, linear_component))

        return transformed_screw

    
    def compute_twist_error(self, T_sb: np.ndarray, T_sd: np.ndarray):
        """
        Computes the twist error matrix [V]. It's a 4x4 matrix with omega as the
        3x3 rotation component and v as the 3x1 linear component.
        """
        twist_error = np.zeros((4, 4))

        # Find relative transformation from end-effector to destination
        T_bd = np.linalg.inv(T_sb) @ T_sd

        # Extract the rotation matrix
        R = T_bd[:3, :3]

        # Extract the displacement vector
        p = T_bd[:3, 3]

        # Now we compute the error twist using the matrix logarithm.
        # First we compute the twist angle. This is the single rotation angle error.
        
        # This is very unstable due to noise that could make it 1.000001. Clip it.
        acos_arg = np.clip((np.linalg.trace(R) - 1) / 2, -1.0, 1.0)
        
        theta = np.clip(acos(acos_arg), -pi, pi)

        # Now we need to handle the two special cases of theta and the general case.
        
        # If theta = 0, then we have pure translation (R = I and rotation error is 0).
        if theta < 1e-6:
            twist_error[:3, 3] = p

        # If theta = pi, we have 180 degree rotation. The goal of this case is to find the most
        # mathematically stable solution. We want to divide by the largest possible number
        # so the solution doesn't shoot off into infinity.
        elif abs(theta - pi) < FloatConstants.THETA_THRESHOLD:
            R_diagonals = np.array([R[0, 0], R[1, 1], R[2, 2]])
            max_diag_idx = np.argmax(R_diagonals)   # returns index of max value

            if max_diag_idx == 0:
                w_skew = (1 / np.sqrt(2 * (1 + R[0,0]))) * np.array([1 + R[0,0], R[1,0], R[2,0]])
            elif max_diag_idx == 1:
                w_skew = (1 / np.sqrt(2 * (1 + R[1,1]))) * np.array([R[0,1], 1 + R[1,1], R[2,1]])
            else:
                w_skew = (1 / np.sqrt(2 * (1 + R[2,2]))) * np.array([R[0,2], R[1,2], 1 + R[2,2]])
   
        # General case
        else:
            w_skew = (1 / (2 * sin(theta))) * (R - R.transpose())

            # linear velocity. Handle case where theta is small but non-zero
            if theta < FloatConstants.THETA_THRESHOLD:
                v = p
            else:
                alpha = (1.0 - (theta/2.0) / tan(theta/2.0)) / (theta**2)
                v = ((1/theta) * np.eye(3) - (1/2) * w_skew + (alpha * (w_skew @ w_skew))) @ p

            # Slot w and v into a 4x4 matrix.
            twist_error[:3, :3] = w_skew * theta
            twist_error[:3, 3] = v
        
        return twist_error
    
    def space_jacobian(self, curr_joint_angles: list[float]):
        """
        TODO: Check the math here.
        """
        jacobian = np.zeros((6, PhysicalParams.NUM_JOINTS))

        jacobian[:,0] = Joint.SHOULDER_PAN.screw_axis

        for idx, (joint, theta) in enumerate(zip(Joint, curr_joint_angles)):
            if idx > 0:
                rot_mat = self.compute_rodrigues_rot(joint.axis, theta)
                p_vec = self.compute_p_vector(joint.axis, theta,joint.lin_velocity)

                ad_map = self.adjoint_map(rot_mat, p_vec)

                jacobian[:,idx] = ad_map @ joint.screw_axis

        return jacobian
    
    def body_jacobian(self, exponentials: list[np.ndarray]):
        """
        We build the Jacobian backwards.

        J_n-1 uses Ad(n)
        J_n-2 uses Ad(n-1)
        """
        T = np.eye(4)
        jacobian = np.zeros((6, PhysicalParams.NUM_JOINTS))

        jacobian[:, -1] = self.BODY_SCREWS[-1]

        for i in range(PhysicalParams.NUM_JOINTS - 2, -1, -1):
            inv_exp = np.linalg.inv(exponentials[i + 1])

            T = T @ inv_exp

            omega = self.BODY_SCREWS[i][:3]
            v = self.BODY_SCREWS[i][3:]

            transformed_screw = self.adjoint_transform(T[:3, :3], T[:3, 3], v, omega)

            jacobian[:, i] = np.transpose(transformed_screw)

        return jacobian
    
    # ==========
    # KINEMATICS
    # ==========
    
    def space_forward_kinematics(self, joint_angles: list[float]):
        """
        Compute the transformation matrix of the end effector given the current
        joint angles.

        Pass a list of joint angles in radians.

        Returns a 4x4 transformation matrix for the current end-effector pose
        relative to the space frame.
        """

        # We initialize the transform as the identity matrix.
        transform = np.eye(4)

        # Compute matrix exponentials and multiply them in sequence
        for joint, angle in zip(Joint, joint_angles):
            transform = transform @ self.compute_matrix_exp(joint.axis, angle, joint.lin_velocity)
        
        # Multiply final transformation matrix and the zero position matrix
        current_position = transform @ PositionConstants.ZERO_TF
        
        return current_position
    
    def body_forward_kinematics(self, joint_angles: list[float]):
        """
        Compute the transformation matrix of the end effector given the current
        joint angles.

        Pass a list of joint angles in radians.

        Returns a 4x4 transformation matrix for the current end-effector pose
        relative to the body frame.
        """
        # We initialize the transform as the identity matrix.
        exponentials = []
        transform = np.eye(4)

        # Compute matrix exponentials and multiply them in sequence
        for screw_axis, angle in zip(self.BODY_SCREWS, joint_angles):
            omega = np.transpose(screw_axis[:3])
            lin_velocity = np.transpose(screw_axis[3:])

            matrix_exp = self.compute_matrix_exp(omega, angle, lin_velocity)
            exponentials.append(matrix_exp)
            transform =  transform @ matrix_exp
        
        # Multiply final transformation matrix and the zero position matrix
        current_position = PositionConstants.ZERO_TF @ transform

        return exponentials, current_position
    
    def inv_kinematics(
            self,
            target_tf: np.ndarray,
            joint_angles: list[float]
        ):
        """
        Inverse kinematics using Newton-Raphson method. Running this method once represents
        a single iteration.
        """
        exp, T_sb = self.body_forward_kinematics(joint_angles)
        current_jacobian = self.body_jacobian(exp)
        pseudo_inv_jacobian = np.linalg.pinv(current_jacobian)
        twist_error = self.compute_twist_error(T_sb, target_tf)

        twist_error_6D = self.se3_to_twist(twist_error, 'v')
        rotation_error = np.linalg.norm(twist_error_6D[:3, 0])
        translation_error = np.linalg.norm(twist_error_6D[3:6, 0])

        angles_increment = pseudo_inv_jacobian @ twist_error_6D

        return angles_increment, rotation_error, translation_error
    
    # =========================
    # GEOMETRIC TRANSFORMATIONS
    # =========================

    def rel_trans_xyz(self, target_coords: tuple[float], current_tf: np.ndarray):
        """
        Linear translation in space, relative to the body frame.
        """

        trans_tf = np.array([
            [1, 0, 0, target_coords[0]],
            [0, 1, 0, target_coords[1]],
            [0, 0, 1, target_coords[2]],
            [0, 0, 0, 1]
        ])

        return trans_tf @ current_tf
    
    def global_trans_xyz(self, target_coords: tuple[float], current_tf: np.ndarray):
        """
        Linear translation in space, relative to the global frame.
        """

        trans_tf = np.array([
            [1, 0, 0, target_coords[0]],
            [0, 1, 0, target_coords[1]],
            [0, 0, 1, target_coords[2]],
            [0, 0, 0, 1]
        ])

        return current_tf @ trans_tf

    def rot_x(self, theta: float, current_tf: np.ndarray):
        """
        Rotation about the x-axis of the body frame.
        """

        Rx = np.array([
            [1, 0, 0, 0],
            [0, cos(theta), -sin(theta), 0],
            [0, sin(theta), cos(theta), 0],
            [0, 0, 0, 1]
        ])

        return current_tf @ Rx
    
    def rot_y(self, theta: float, current_tf: np.ndarray):
        """
        Rotation about the y-axis of the body frame.
        """
        
        Ry = np.array([
            [cos(theta), 0, sin(theta), 0],
            [0, 1, 0, 0],
            [-sin(theta), 0, cos(theta), 0],
            [0, 0, 0, 1]
        ])

        return current_tf @ Ry

    def rot_z(self, theta: float, current_tf: np.ndarray):
        """
        Rotation about the z-axis of the body frame.
        """

        Rz = np.array([
            [cos(theta), -sin(theta), 0, 0],
            [sin(theta), cos(theta), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        return current_tf @ Rz

def main():
    k = Kinematics()

if __name__=='__main__':
    main()