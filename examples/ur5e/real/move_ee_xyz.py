import time

from airobot import Robot


def main():
    """
    Move the robot end effector in a straight line.

    The pb=False flag is set because we are using the real robot
    (pb -- pybullet).

    First movement is executed with MoveIt!, and second movement
    is executed using urscript commands.
    """
    robot = Robot('ur5e_2f140',
                  pb=False,
                  use_cam=False)
    robot.arm.go_home()
    robot.arm.move_ee_xyz([0.2, 0.0, 0.0], wait=True)
    time.sleep(1.0)

    robot.arm.set_comm_mode(use_urscript=True)
    robot.arm.move_ee_xyz([-0.2, 0.0, 0.0], wait=True)


if __name__ == '__main__':
    main()
