import time

import airobot as ar


def main():
    """
    This function demonstrates how to perform torque
    control on the simulated UR robot
    """
    robot = ar.create_robot('yumi',
                            robot_cfg={'render': True,
                                       'self_collision': True})
    robot.go_home()
    arm = 'left'
    print("waiting for arm to settle...")
    time.sleep(5.0)
    robot.enable_torque_control()
    print("starting random torques on one arm...")
    while True:
        robot.set_jtorq([100, 100, 100, 20, 20, 20, 20], arm=arm)
        time.sleep(0.0001)


if __name__ == '__main__':
    main()
