import time

import airobot as ar
import numpy as np


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions.
    """
    robot = ar.Robot('yumi_palms')
    robot.arm.go_home()
    robot.arm.right_arm.set_jpos(
        [0.5, -1.0, -1.0, -0.0, -0.2, 1.0, -1.57])

    time.sleep(3)
    ar.log_info('Right arm joint positions: ')
    ar.log_info(np.round(robot.arm.right_arm.get_jpos(), 6))
    ar.log_info('Both arm joint positions: ')
    ar.log_info(np.round(robot.arm.get_jpos(), 6))
    ar.log_info('\n')

    robot.arm.right_arm.set_jpos(0.5, 'yumi_joint_3_r')
    time.sleep(3)

    robot.arm.set_jpos(
        [0.05, -1.33, 0.87, 0.94, 0.41, 0.26, -2.01],
        arm='left',
        ignore_physics=True)

    time.sleep(3)
    ar.log_info('Left arm joint positions: ')
    ar.log_info(np.round(robot.arm.left_arm.get_jpos(), 6))
    ar.log_info('Both arm joint positions: ')
    ar.log_info(np.round(robot.arm.get_jpos(), 6))
    ar.log_info('\n')

    robot.arm.set_jpos(0.5,
                       joint_name='yumi_joint_3_l',
                       arm='left',
                       ignore_physics=True)
    time.sleep(3)

    robot.arm.go_home(ignore_physics=True)


if __name__ == '__main__':
    main()
