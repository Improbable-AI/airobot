import airobot as ar

def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions

    The pb=False flag is set because we are using the real robot (pb -- pybullet)
    """
    home_pos = [1.57, -1.5, 2.0, -2.05, -1.57, 0]

    robot = ar.create_robot('ur5e', pb=False)
    robot.set_comm_mode(use_urscript=True)

    current_pos = robot.get_jpos()
    goal_pos = current_pos
    goal_pos[0] += 0.1

    robot.set_jpos(goal_pos)

    # robot.stop()

if __name__ == '__main__':
    main()
