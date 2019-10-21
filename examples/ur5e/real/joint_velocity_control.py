import math
import time

import rospy

from airobot import Robot


def sin_wave(t, f, A):
    """
    Return the sine-wave value at time step t

    Args:
        t (float): time
        f (float): frequency
        A (float): amplitude

    Returns:
        a sin-wave value

    """
    return A * math.cos(2 * math.pi * f * t)


def main():
    """
    Move all the joints of the robot in a sine-wave fashion

    The pb=False flag is set because we are using the real robot
    (pb -- pybullet)
    """
    robot = Robot('ur5e',
                  pb=False,
                  use_cam=False)

    robot.arm.go_home()
    robot.arm.set_comm_mode(use_urscript=True)

    A = 0.1
    f = 0.3
    start_time = time.time()
    while not rospy.is_shutdown():
        elapsed_time = time.time() - start_time
        vels = [sin_wave(elapsed_time, f, A)] * robot.arm.arm_dof
        robot.arm.set_jvel(vels, wait=True)
        time.sleep(0.001)


if __name__ == '__main__':
    main()
