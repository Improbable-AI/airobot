import time

import airobot as ar


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions
    """
    robot = ar.create_robot('yumi',
                            robot_cfg={'render': True, 'self_collision': True})
    from IPython import embed
    embed()
    robot.go_home()
    robot.set_jpos([-0.8, -1.2, -2.2, -1.5, 2.0, 0, 0])
    # sleep statement is not necessary
    time.sleep(3)
    robot.set_jpos(0.5, 'yumi_joint_1_r')
    time.sleep(3)


if __name__ == '__main__':
    main()
