import time

import airobot as ar


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions

    The pb=False flag is set because we are using the real robot (pb -- pybullet)
    """
    robot = ar.create_robot('ur5e', pb=False)
    current_pos = robot.get_jpos()
    goal_pos = current_pos 
    goal_pos[0] += 0.1
    robot.set_jpos(goal_pos, wait=False)
    # sleep statement is not necessary
    time.sleep(1)
    robot.set_jpos(0.5, 'shoulder_pan_joint')
    time.sleep(1)


if __name__ == '__main__':
    main()
