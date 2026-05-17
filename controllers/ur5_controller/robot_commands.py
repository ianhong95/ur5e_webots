"""
This set of objects store the functions to call when a motion function is called.

Each class corresponds to a type of motion that is passed to the motion queue.
"""

import numpy as np

class RobotCmd:
    """
    Blueprint for the common interface. If you create a class that inherits
    from this and forget to write the execute() method, it will run the default
    execute() method which raises an error. Every child MUST override this.
    """
    def execute(self, robot_motion, T_sb_start):
        raise NotImplementedError
    
class MoveRelCmd(RobotCmd):
    def __init__(self, x:float=0.0, y:float=0.0, z:float=0.0):
        self.move_vector = np.array([x, y, z])

    def execute(self, robot_motion, T_sb_start):
        robot_motion.T_sd = robot_motion.controller.rel_trans_xyz(self.move_vector, T_sb_start)

class MoveAbsCommand(RobotCmd):
    def __init__(self, target_tf: np.ndarray):
        self.target_tf = target_tf

    def execute(self, robot_motion, T_sb_start: np.ndarray):
        robot_motion.T_sd = self.target_tf
        robot_motion.t_elapsed = 0.0
        robot_motion.compute_vel_path(T_sb_start, self.target_tf)
        robot_motion.is_moving = True

class GripperCmd(RobotCmd):
    def __init__(self, target_gripper_position: float):
        self.target_gripper_position = target_gripper_position

    def execute(self, robot_motion, T_sb_start: np.ndarray):
        robot_motion.target_gripper_position = self.target_gripper_position
        robot_motion.controller.actuate_gripper(self.target_gripper_position)
