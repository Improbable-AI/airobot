import time

import airobot as ar
import copy

def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions.

    The pb=False flag (in ar.create_robot())is set because
    we are using the real robot (pb -- pybullet).

    The robot starts at home, and moves to a new set of joint
    angles with comm_mode use_urscript=False.

    The robot then switches comm_mode to use_urscript=True,
    which means it won't use ROS or MoveIt to execute it's commands.

    The robot finally moves to a new position using direct joint
    position control. This second movement has been verified to be
    collision free (change the goal positions when use_urscript=True
    at your own risk).
    """

    robot = ar.Robot('yumi_palms', pb=False, use_cam=False)

    # TODO: how to make the robot go home safely without moveit?

    from IPython import embed
    embed()

    current_pos = robot.arm.get_jpos()
    current_ee_pose = robot.arm.get_ee_pose()

    time.sleep(3.0)
    ind = 6
    print('Setting goal pos at index %d, small delta away from current configuation' % ind)

    goal_pos = copy.deepcopy(current_pos) * 180 / np.pi
    goal_pos[ind] += 0.1

    print("Joint Angles: ")
    print(current_pos)
    print('End Effector Pose: ')
    print(current_ee_pose)
    print('Target Angles: ')
    print(goal_pos)

    # robot.arm.set_jpos(goal_pos, wait=False)

    time.sleep(3.0)


if __name__ == '__main__':
    main()
