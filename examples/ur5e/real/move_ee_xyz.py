import airobot as ar


def main():
    """
    Move the robot end effector in a straight line

    The pb=False flag is set because we are using the real robot
    (pb -- pybullet)    

    First movement is executed using MoveIt (use_urscript=False)

    Second movement back to home is used using direct
    cartesian end effector commands (use_urscript=True)

    Movements executed without collision checking have been verified
    to be collision free (modify commands executed with 
    use_urscript=True at your own risk!)
    """
    robot = ar.create_robot('ur5e', pb=False)
    robot.go_home()
    robot.move_ee_xyz([0.2, 0.0, 0.0], wait=True)

    print('switching mode to use_urscript=True')
    robot.set_comm_mode(use_urscript=True)
    robot.move_ee_xyz([-0.2, 0.0, 0.0], wait=True)

    robot.close()


if __name__ == '__main__':
    main()
