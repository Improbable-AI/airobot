import time

import airobot as ar


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions
    """
    robot = ar.create_robot('ur5e',
                            robot_cfg={'render': True})
    robot.go_home()
    robot.set_jpos([-0.8, -1.2, -2.2, -1.5, 2.0, 0])
    # sleep statement is not necessary
    time.sleep(3)
    robot.set_jpos(0.5, 'shoulder_pan_joint')
    time.sleep(3)


if __name__ == '__main__':
    main()
