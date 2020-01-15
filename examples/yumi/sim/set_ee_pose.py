import time

from airobot import Robot


def main():
    """
    Move the robot end effector to the desired pose
    """
    robot = Robot('yumi_grippers', arm_cfg={'render': True})
    robot.arm.go_home()
    tgt_pos = [0.35, 0.0, 0.2]
    robot.arm.set_ee_pose(tgt_pos, arm='right')

    # sleep statement is not necessary
    time.sleep(3)
    tgt_pos = [0.3, -0.15, 0.25]
    tgt_euler = [-0.5, 0.0, 0.0]
    robot.arm.set_ee_pose(tgt_pos, tgt_euler, arm='right')
    time.sleep(3)
    pos, quat, rot, euler = robot.arm.get_ee_pose(arm='right')
    print('End effector pose:')
    print('Position:')
    print(pos)
    print('Euler angles:')
    print(euler)


if __name__ == '__main__':
    main()
