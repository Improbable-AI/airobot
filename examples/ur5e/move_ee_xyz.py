import time

import airobot as ar


def main():
    robot = ar.create_robot('ur5e', robot_cfg={'render': True})
    robot.go_home()
    robot.move_ee_xyz([0.1, 0.1, 0.1])
    time.sleep(3)


if __name__ == '__main__':
    main()
