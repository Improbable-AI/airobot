from yacs.config import CfgNode as CN

_C = CN()
_C.ROBOT_DESCRIPTION = '/robot_description'
_C.PYBULLET_URDF = 'yumi_gelsight.urdf'

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.CLASS = 'Yumi'
_C.MOVEGROUP_NAME = 'manipulator'
_C.ROSTOPIC_JOINT_STATES = '/joint_states'

# base frame for the arm
_C.ROBOT_BASE_FRAME = 'yumi_body'

_C.JOINT_SPEED_TOPIC = '/joint_speed'

# inverse kinematics position tolerance (m)
_C.IK_POSITION_TOLERANCE = 0.01
# inverse kinematics orientation tolerance (rad)
_C.IK_ORIENTATION_TOLERANCE = 0.1
_C.MAX_JOINT_ERROR = 0.01
_C.MAX_JOINT_VEL_ERROR = 0.1
_C.MAX_EE_POS_ERROR = 0.01
# real part of the quaternion difference should be
# greater than 1-error
_C.MAX_EE_ORI_ERROR = 0.02
_C.TIMEOUT_LIMIT = 10

# reset position for the robot in pybullet
_C.PYBULLET_RESET_POS = [0, 0, -0.1]
# reset orientation (euler angles) for the robot in pybullet
_C.PYBULLET_RESET_ORI = [0, 0, 0]

_C.COMPLIANT_GAINS = [0.1, 0.1]


def get_yumi_arm_cfg():
    return _C.clone()
