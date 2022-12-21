import time

import airobot as ar
import numpy as np


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions.

    The pb=False flag (in ar.create_robot())is set because
    we are using the real robot (pb -- pybullet).

    The robot starts at home, and moves to a new set of joint
    angles with comm_mode use_urscript=False.

    The robot then switches comm_mode to use_urscript=True,
    which means it won't use ROS or MoveIt to execute it's commands.

    The robot finally moves to a new position using direct joint
    position control. This second movement has been verified to be
    collision free (change the goal positions when use_urscript=True
    at your own risk).
    """
    robot = ar.Robot('ur5e_2f140', pb=False, use_cam=False)
    robot.arm.go_home()

    goal_pos = np.deg2rad([-34, -74, 77.1, -141.1, -89, 0])


    robot.arm.set_jpos(goal_pos, wait=True)
    print("Joint Angles: ")
    print(robot.arm.get_jpos())

    goal_pos = np.deg2rad([6.38, -49.51, 68.9, -137.61, -64.52, 0])

    robot.arm.set_jpos(goal_pos, wait=True)
    print("Joint Angles: ")
    print(robot.arm.get_jpos())


if __name__ == '__main__':
    main()
