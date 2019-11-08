import numpy as np

from airobot import Robot


def main():
    """
    This function demonstrates how to get joint information
    such as joint positions/velocities/torques.
    """
    robot = Robot('ur5e', arm_cfg={'render': True})
    robot.arm.go_home()
    print('\nJoint positions for all actuated joints:')
    jpos = robot.arm.get_jpos()
    print(np.around(jpos, decimals=3))
    joint = 'shoulder_pan_joint'
    print('Joint [%s] position: %.3f' %
          (joint, robot.arm.get_jpos('shoulder_pan_joint')))
    print('Joint velocities:')
    jvel = robot.arm.get_jvel()
    print(np.around(jvel, decimals=3))
    print('Joint torques:')
    jtorq = robot.arm.get_jtorq()
    print(np.around(jtorq, decimals=3))
    robot.arm.eetool.close()
    print('Gripper position (close): %.3f' % robot.arm.eetool.get_pos())
    robot.arm.eetool.open()
    print('Gripper position (open): %.3f' % robot.arm.eetool.get_pos())


if __name__ == '__main__':
    main()
