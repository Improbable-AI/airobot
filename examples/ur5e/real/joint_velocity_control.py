import math
import time
import sys

import airobot as ar


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
    """
    robot_ip = sys.argv[1]
    robot = ar.create_robot('ur5e', pb=False, robot_cfg={
                            'robot_ip': robot_ip})
    robot.go_home()

    A = 0.4
    f = 0.3
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        vels = [sin_wave(elapsed_time, f, A)] * robot.arm_dof
        robot.set_jvel(vels[0], joint_name='shoulder_pan_joint')
        time.sleep(0.01)


if __name__ == '__main__':
    main()
