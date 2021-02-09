import time
import numpy as np

from airobot import Robot


def main():
    """
    Rotate the end effector about a single axis, one at a time
    """
    robot = Robot('ur5e_2f140')
    robot.arm.go_home()
    robot.arm.rot_ee_xyz(np.pi/4, axis='x')
    time.sleep(1)
    robot.arm.rot_ee_xyz(-np.pi/4, axis='x')
    time.sleep(3)
    
    robot.arm.rot_ee_xyz(np.pi/4, axis='y')
    time.sleep(1)
    robot.arm.rot_ee_xyz(-np.pi/4, axis='y')
    time.sleep(3)

    robot.arm.rot_ee_xyz(np.pi/4, axis='z')
    time.sleep(1)
    robot.arm.rot_ee_xyz(-np.pi/4, axis='z')
    time.sleep(3)


if __name__ == '__main__':
    main()
