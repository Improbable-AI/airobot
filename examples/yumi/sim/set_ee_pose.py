import time

import airobot as ar


def main():
    """
    Move the robot end effector to the desired pose
    """
    robot = ar.create_robot('yumi',
                            robot_cfg={'render': True, 'self_collision': True})
    robot.go_home()
    
    arm = 'right'
    
    tgt_pos = [0.5, 0.3, 1.4]
    robot.set_ee_pose(tgt_pos, arm=arm)

    # sleep statement is not necessary
    time.sleep(1)
    tgt_pos = [0.5, -0.3, 1.4]
    tgt_euler = [0.4, 0.2, 0.3]
    robot.set_ee_pose(tgt_pos, tgt_euler, arm=arm)
    time.sleep(1)
    pos, quat, rot, euler = robot.get_ee_pose(arm=arm)
    print('End effector pose:')
    print('Position:')
    print(pos)
    print('Euler angles:')
    print(euler)

    arm = 'left'

    tgt_pos = [0.5, 0.3, 1.4]
    robot.set_ee_pose(tgt_pos, arm=arm)

    # sleep statement is not necessary
    time.sleep(1)
    tgt_pos = [0.5, -0.3, 1.4]
    tgt_euler = [0.4, 0.2, 0.3]
    robot.set_ee_pose(tgt_pos, tgt_euler, arm=arm)
    time.sleep(1)
    pos, quat, rot, euler = robot.get_ee_pose(arm=arm)
    print('End effector pose:')
    print('Position:')
    print(pos)
    print('Euler angles:')
    print(euler)


if __name__ == '__main__':
    main()
