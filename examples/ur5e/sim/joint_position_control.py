import time

from airobot import Robot


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions
    """
    robot = Robot('ur5e', arm_cfg={'render': True})
    robot.arm.go_home()
    robot.arm.set_jpos([-0.8, -1.2, -2.2, -1.5, 2.0, 0])
    # sleep statement is not necessary
    time.sleep(3)
    robot.arm.set_jpos(0.5, 'shoulder_pan_joint')
    time.sleep(3)


if __name__ == '__main__':
    main()
