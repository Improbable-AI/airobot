import airobot as ar
import time

def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions

    The pb=False flag is set because we are using the real robot (pb -- pybullet)
    """
    print("creating robot and activating gripper...")
    robot = ar.create_robot('ur5e', pb=False)
    robot.gripper.activate()
    time.sleep(1.0)

    print("setting comm mode...")
    robot.set_comm_mode(use_urscript=True)

    print("getting current and goal pos...")
    current_pos = robot.get_jpos()
    goal_pos = current_pos
    goal_pos[-1] += 0.2

    print("setting goal pos...")
    robot.set_jpos(goal_pos, wait=False)

    # print("done")
    # robot.stop()

if __name__ == '__main__':
    main()
