import time

import airobot as ar


def main():
    robot = ar.create_robot('ur5e',
                            robot_cfg={'render': True,
                                       'self_collision': True})
    robot.go_home()
    robot.enable_torque_control()
    while True:
        robot.set_jtorq([100, 100, 100, 20, 20, 20])
        time.sleep(0.0001)


if __name__ == '__main__':
    main()
