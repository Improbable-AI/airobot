import airobot as ar
import time


def main():
    """
    Move the robot end effector in a straight line

    The pb=False flag is set because we are using the real robot
    (pb -- pybullet)

    Movement is executed with MoveIt!
    """
    robot = ar.create_robot('ur5e', pb=False)
    robot.go_home()
    robot.move_ee_xyz([0.2, 0.0, 0.0], wait=True)


if __name__ == '__main__':
    main()
