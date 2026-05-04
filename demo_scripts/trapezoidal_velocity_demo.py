from controllers.ur5_controller.robot_motion import RobotMotion
from controllers.ur5_controller.ur5_definitions import Gripper
from demo_scripts.test_positions import Positions

def main():
    robot = RobotMotion()
    robot._list_devices()
    robot.moveL(Positions.HOME)
    
    while True:
        robot.move_x(-150)
        robot.move_y(200)
        robot.move_x(150)
        robot.move_y(-400)
        robot.move_x(-150)
        robot.move_z(150)
        robot.set_gripper(Gripper.LEFT_FINGER.max)
        robot.set_gripper(Gripper.LEFT_FINGER.min)
        robot.move_z(-150)
        robot.move_y(200)
        robot.move_x(150)

if __name__ == '__main__':
    main()