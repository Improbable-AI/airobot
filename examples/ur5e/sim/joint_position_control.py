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
    robot.set_jpos([-0.8, -0.6, 1.8, -2.65, -1.57, 0])
    # sleep statement is not necessary
    time.sleep(1)
    robot.set_jpos(0.5, 'shoulder_pan_joint')
    time.sleep(1)


if __name__ == '__main__':
    main()
