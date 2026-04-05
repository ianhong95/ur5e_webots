from robot_motion import RobotMotion

def main():
    robot = RobotMotion()

    robot.set_joint_angles(robot.DEFAULT_POSITIONS['HOME'])
    robot.go_to_position(robot.TEST_TARGETS['TEST'])
    _, fk = robot.k.body_forward_kinematics(robot.joint_angles) 
    
    while True:
        robot.move_x(-150)
        robot.move_y(200)
        robot.move_x(150)
        robot.move_y(-400)
        robot.move_x(-150)
        robot.move_z(150)
        robot.move_z(-150)
        robot.move_y(200)
        # robot.rot_y(pi/4)
        # robot.rot_y(-pi/4)
        robot.move_x(150)

if __name__ == '__main__':
    main()