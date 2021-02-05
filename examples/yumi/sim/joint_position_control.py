import time

import airobot as ar
import numpy as np


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions.
    """
    robot = ar.Robot('yumi_grippers')
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

    robot.arm.left_arm.eetool.activate()

    robot.arm.left_arm.eetool.open()
    time.sleep(3)

    robot.arm.left_arm.eetool.set_pos(0.005)
    ar.log_info('Left gripper position: ')
    ar.log_info(np.round(robot.arm.left_arm.eetool.get_pos(), 6))
    ar.log_info('\n')
    time.sleep(3)

    robot.arm.left_arm.eetool.close()

    time.sleep(3)
    ar.log_info('Left gripper position at close: ')
    ar.log_info(np.round(robot.arm.left_arm.eetool.get_pos(), 6))
    ar.log_info('\n')


if __name__ == '__main__':
    main()
