import time
import sys

import airobot as ar


def rad2deg(joint_values):
    deg_values = []
    for i, val in enumerate(joint_values):
        deg_values.append(val * (180/3.1415))

    return deg_values


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions

    The pb=False flag is set because we are using the real robot (pb -- pybullet)
    """
    # make it an command line argument
    home_pos = [1.57, -1.5, 2.0, -2.05, -1.57, 0]
    robot_ip = str(sys.argv[1])

    robot = ar.create_robot('ur5e', pb=False, robot_cfg={'robot_ip': robot_ip})
    
    robot.close()

if __name__ == '__main__':
    main()
