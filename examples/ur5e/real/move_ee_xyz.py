import airobot as ar


def main():
    """
    Move the robot end effector in a straight line
    """
    robot = ar.create_robot('ur5e', pb=False)
    robot.go_home()
    robot.move_ee_xyz([0.2, 0.0, 0.0], wait=True)

    robot.set_comm_mode(use_urscript=True)
    robot.move_ee_xyz([-0.2, 0.0, 0.0], wait=True)

    robot.close()


if __name__ == '__main__':
    main()
