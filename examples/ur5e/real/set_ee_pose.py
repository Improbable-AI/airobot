import time

import airobot as ar


def main():
    """
    Move the robot end effector to the desired pose
    """
    robot = ar.create_robot('ur5e', pb=False, robot_cfg={
                            'host': '128.30.31.59'})
    robot.go_home()
    current_pos = robot.get_ee_pose()
    print("current EE pose: ")
    print(current_pos)

    goal_pos = current_pos[0]
    goal_ori = current_pos[-1]

    # deltas specified in cm
    # delta_x = 3
    # delta_y = 3
    # delta_z = 3

    # goal_pos[0] -= delta_x * (0.01) #
    # goal_pos[1] -= delta_y * (0.01)
    # goal_pos[2] -= delta_z * (0.01)

    # print("goal EE pose: ")
    # print(goal_pos)
    # robot.set_ee_pose(goal_pos)

    # deltas specified in rad
    delta_rx = 0.1
    delta_ry = 0.1
    delta_rz = 0.1

    goal_ori[0] -= delta_rx
    goal_ori[1] -= delta_ry
    goal_ori[2] -= delta_rz

    print("goal EE pose: ")
    print(goal_pos, goal_ori)
    robot.set_ee_pose(goal_pos, goal_ori)

    # tgt_pos = [0.5, 0.3, 1.4]
    # robot.set_ee_pose(tgt_pos)

    # sleep statement is not necessary
    # time.sleep(1)
    # tgt_pos = [0.5, -0.3, 1.4]
    # tgt_euler = [0.4, 0.2, 0.3]
    # robot.set_ee_pose(tgt_pos, tgt_euler)
    # time.sleep(1)
    # pos, quat, rot, euler = robot.get_ee_pose()
    # print('End effector pose:')
    # print('Position:')
    # print(pos)
    # print('Euler angles:')
    # print(euler)

    robot.close()


if __name__ == '__main__':
    main()