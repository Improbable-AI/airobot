import time

from airobot import Robot


def main():
    """
    This function demonstrates how to perform torque
    control on the simulated UR robot
    """
    robot = Robot('ur5e', arm_cfg={'render': True,
                                   'self_collision': True})
    robot.arm.go_home()
    robot.arm.enable_torque_control()
    while True:
        robot.arm.set_jtorq([100, 100, 100, 20, 20, 20])
        time.sleep(0.0001)


if __name__ == '__main__':
    main()
