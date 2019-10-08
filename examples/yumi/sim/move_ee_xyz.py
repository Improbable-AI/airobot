import time

import airobot as ar


def main():
    """
    Move the robot end effector in a straight line
    """
    robot = ar.create_robot('yumi', robot_cfg={'render': True, 'self_collision': True})
    robot.go_home()
    robot.move_ee_xyz([0.0, 0.0, 0.0], arm='right')

    robot.go_home()
    robot.move_ee_xyz([0.01, 0.0, 0.0], arm='left')

    time.sleep(3)


if __name__ == '__main__':
    main()
