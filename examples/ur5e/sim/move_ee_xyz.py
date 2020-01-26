import time

from airobot import Robot


def main():
    """
    Move the robot end effector in a straight line
    """
    robot = Robot('ur5e')
    robot.arm.go_home()
    robot.arm.move_ee_xyz([0.1, 0.1, 0.1])
    time.sleep(3)


if __name__ == '__main__':
    main()
