import time

import airobot as ar


def main():
    """
    Move the robot end effector to the desired pose
    """
    robot = ar.create_robot('ur5e',
                            robot_cfg={'render': True})
    robot.go_home()
    tgt_pos = [0.5, 0.3, 1.4]
    robot.set_ee_pose(tgt_pos)

    # sleep statement is not necessary
    time.sleep(1)
    tgt_pos = [0.5, -0.3, 1.4]
    tgt_euler = [0.4, 0.2, 0.3]
    robot.set_ee_pose(tgt_pos, tgt_euler)
    time.sleep(1)
    pos, quat, rot, euler = robot.get_ee_pose()
    print('End effector pose:')
    print('Position:')
    print(pos)
    print('Euler angles:')
    print(euler)


if __name__ == '__main__':
    main()
