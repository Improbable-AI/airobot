import time

import airobot as ar

def rad2deg(joint_values):
    deg_values = []
    for i, val in enumerate(joint_values):
        deg_values.append(val * (180/3.1415))

    return deg_values

def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions

    The pb=False flag is set because we are using the real robot (pb -- pybullet)
    """
    home_pos = [1.57, -1.5, 2.0, -2.05, -1.57, 0]

    robot = ar.create_robot('ur5e', pb=False, robot_cfg={'host': '128.30.31.59'})

    current_pos = robot.get_jpos()
    print("Current joint configuration (degrees): ")
    print(rad2deg(current_pos))

    robot.go_home()
    current_pos = robot.get_jpos()
    print("Current joint configuration (degrees): ")
    print(rad2deg(current_pos))
    time.sleep(1)

    goal_pos = current_pos 
    goal_pos[0] += 0.1

    # goal_pos[0] = home_pos[0]
    # goal_pos[1] = home_pos[1]
    # goal_pos[2] = home_pos[2]
    # goal_pos[3] = home_pos[3]
    # goal_pos[4] = home_pos[4]
    # goal_pos[5] = home_pos[5]

    # print("Goal joint configuration (degrees): ")
    # print(rad2deg(goal_pos))

    robot.set_jpos(goal_pos, wait=False)
    time.sleep(1)

    # robot.set_jpos(goal_pos[0], 'shoulder_pan_joint')
    # robot.set_jpos(goal_pos[1], 'shoulder_lift_joint')
    # robot.set_jpos(goal_pos[2], 'elbow_joint')
    # robot.set_jpos(goal_pos[3], 'wrist_1_joint')
    # robot.set_jpos(goal_pos[4], 'wrist_2_joint')
    # robot.set_jpos(goal_pos[5], 'wrist_3_joint')

    time.sleep(1)
    
    robot.close()



if __name__ == '__main__':
    main()
