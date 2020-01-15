import time

from airobot import Robot


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions
    """
    robot = Robot('yumi_grippers', arm_cfg={'render': True})
    robot.arm.go_home()

    robot.arm.right_arm.set_jpos(
        [0.5, -1.0, -1.0, -0.0, -0.2, 1.0, -1.57])

    time.sleep(3)
    print('Right arm joint positions: ')
    print(robot.arm.right_arm.get_jpos())
    print('Both arm joint positions: ')
    print(robot.arm.get_jpos())
    print('\n')

    robot.arm.right_arm.set_jpos(0.5, 'yumi_joint_3_r')
    time.sleep(3)

    robot.arm.left_arm.eetool.activate()

    robot.arm.left_arm.eetool.open()
    time.sleep(3)

    robot.arm.left_arm.eetool.set_pos(0.005)
    print('Left gripper position: ')
    print(robot.arm.left_arm.eetool.get_pos())
    print('\n')
    time.sleep(3)

    robot.arm.left_arm.eetool.close()
    time.sleep(3)
    print('Left gripper position at close: ')
    print(robot.arm.left_arm.eetool.get_pos())
    print('\n')


if __name__ == '__main__':
    main()
