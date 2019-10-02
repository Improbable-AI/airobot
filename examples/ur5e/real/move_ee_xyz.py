import time
import sys

import airobot as ar


def main():
    """
    Move the robot end effector in a straight line
    """
    robot_ip = sys.argv[1]
    robot = ar.create_robot('ur5e', pb=False, robot_cfg={
                            'robot_ip': robot_ip})
    robot.go_home()
    robot.move_ee_xyz([0.2, 0.0, 0.0])
    robot.move_ee_xyz([-0.2, 0.0, 0.0])
    # time.sleep(3)
    robot.close()


if __name__ == '__main__':
    main()
