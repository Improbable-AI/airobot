import time

import airobot as ar
from airobot import Robot


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions

    The pb=False flag (in ar.create_robot())is set because
    we are using the real robot (pb -- pybullet)

    The robot starts at home, and moves to a new set of joint
    angles with comm_mode use_urscript=False

    The robot then switches comm_mode to use_urscript=True,
    which means it won't use ROS or MoveIt to execute it's commands

    The robot finally moves to a new position using direct joint
    position control. This second movement has been verified to be
    collision free (change the goal positions when use_urscript=True
    at your own risk)
    """
    robot = Robot('ur5e_2f140', pb=False, use_cam=False)
    robot.arm.go_home()

    goal_pos = [0.5, -2, -1.1, -0.95, 1.7, -0.1]

    robot.arm.set_jpos(goal_pos, wait=True)
    print("Joint Angles: ")
    print(robot.arm.get_jpos())

    ar.utils.common.print_red(
        """
        ---SWITCHING COMM MODE TO USE_URSCRIPT=TRUE---\n\n\n
        Robot will execute a joint position command using
        direct position control\n\n\n
        These movements have been verified to be collision free\n\n\n
        ---MODIFY THE GOAL POSITIONS WHEN USING DIRECT POSITION
        CONTROL AT YOUR OWN RISK---
        """
    )
    time.sleep(3.0)

    robot.arm.set_comm_mode(use_urscript=True)

    goal_pos = [0, -1.66, -1.92, -1.12, 1.57, 0]

    robot.arm.set_jpos(goal_pos, wait=True)
    print("Joint Angles: ")
    print(robot.arm.get_jpos())
    print('Joint Angle of Joint[shoulder_pan_joint]:')
    print(robot.arm.get_jpos('shoulder_pan_joint'))

    robot.arm.eetool.activate()
    print('Opening gripper')
    robot.arm.eetool.open()
    time.sleep(1.0)
    print('Closing gripper')
    robot.arm.eetool.close()
    time.sleep(1.0)


if __name__ == '__main__':
    main()
