import airobot as ar
import numpy as np
from airobot import Robot


def main():
    """
    This function demonstrates how to get joint information
    such as joint positions/velocities/torques.
    """
    robot = Robot('ur5e_2f140', pb_cfg={'gui': True})
    robot.arm.go_home()
    ar.log_info('\nJoint positions for all actuated joints:')
    jpos = robot.arm.get_jpos()
    ar.log_info(np.round(jpos, decimals=3))
    joint = 'shoulder_pan_joint'
    ar.log_info('Joint [%s] position: %.3f' %
                (joint, robot.arm.get_jpos('shoulder_pan_joint')))
    ar.log_info('Joint velocities:')
    jvel = robot.arm.get_jvel()
    ar.log_info(np.round(jvel, decimals=3))
    ar.log_info('Joint torques:')
    jtorq = robot.arm.get_jtorq()
    ar.log_info(np.round(jtorq, decimals=3))
    robot.arm.eetool.close()
    ar.log_info('Gripper position (close): %.3f' % robot.arm.eetool.get_pos())
    robot.arm.eetool.open()
    ar.log_info('Gripper position (open): %.3f' % robot.arm.eetool.get_pos())


if __name__ == '__main__':
    main()
