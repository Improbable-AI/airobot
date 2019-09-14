import numpy as np

import airobot as ar


def main():
    robot = ar.create_robot('ur5e', robot_cfg={'render': False})
    robot.go_home()
    print('\nJoint positions for all actuated joints:')
    jpos = robot.get_jpos()
    print(np.around(jpos, decimals=3))
    joint = 'shoulder_pan_joint'
    print('Joint [%s] position: %.3f' %
          (joint, robot.get_jpos('shoulder_pan_joint')))
    print('Joint velocities:')
    jvel = robot.get_jvel()
    print(np.around(jvel, decimals=3))
    print('Joint torques:')
    jtorq = robot.get_jtorq()
    print(np.around(jtorq, decimals=3))


if __name__ == '__main__':
    main()
