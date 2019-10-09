import time

import airobot as ar


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions

    The pb=False flag is set because we are using the real robot
    (pb -- pybullet)

    The robot starts at home, and moves to a new set of joint
    angles with comm_mode use_urscript=False

    The robot then switches comm_mode to use_urscript=True,
    which means it won't use ROS or MoveIt to execute it's commands

    The robot finally moves to a new position using direct joint
    position control. This second movement has been verified to be
    collision free (change the goal positions when use_urscript=True
    at your own risk)
    """
    robot = ar.create_robot('ur5e', pb=False)
    robot.go_home()

    goal_pos = []

    robot.set_jpos(goal_pos, wait=True)
    print("Joint Angles: ")
    print(robot.get_jpos())

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

    robot.set_comm_mode(use_urscript=True)

    goal_pos = []

    robot.set_jpos(goal_pos, wait=True)
    print("Joint Angles: ")
    print(robot.get_jpos())



if __name__ == '__main__':
    main()
