from controllers.ur5_controller.robot_motion import RobotMotion
from demo_scripts.test_positions import Positions, JointPresets
import time

def main():
    robot = RobotMotion()
    robot._list_devices()
    robot.set_joint_angles(JointPresets.HOME)
    
    while True:
        robot.go_to_position(Positions.HOME)
        robot.go_to_position(Positions.TEST2)
        robot.go_to_position(Positions.TEST3)
        robot.go_to_position(Positions.TEST4)
        robot.go_to_position(Positions.TEST5)
        robot.go_to_position(Positions.TEST6)
        robot.go_to_position(Positions.TEST7)
        robot.go_to_position(Positions.TEST8)
        robot.go_to_position(Positions.TEST2)


if __name__ == '__main__':
    main()