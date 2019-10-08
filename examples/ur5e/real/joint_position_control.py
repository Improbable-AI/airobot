import time

import airobot as ar


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions

    The pb=False flag is set because we are using the real robot
    (pb -- pybullet)
    """
    print("creating robot and going to home position...")
    robot = ar.create_robot('ur5e', pb=False)
    robot.go_home()
    time.sleep(1.0)

    print("getting current and nearby goal pos...")
    current_pos = robot.get_jpos()
    goal_pos = current_pos
    goal_pos[-1] += 0.2

    print("setting goal pos...")
    robot.set_jpos(goal_pos, wait=True)

    print("setting new comm mode...")
    robot.set_comm_mode(use_urscript=True)

    print("getting current and goal pos...")
    current_pos = robot.get_jpos()
    goal_pos = current_pos
    goal_pos[-1] -= 0.2

    print("setting goal pos...")
    robot.set_jpos(goal_pos, wait=True)

    print("done")
    robot.close()


if __name__ == '__main__':
    main()
