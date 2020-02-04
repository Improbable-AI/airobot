from yacs.config import CfgNode as CN

_C = CN()
_C.ROBOT_DESCRIPTION = '/robot_description'

_C.ROSTOPIC_JOINT_STATES = '/joint_states'

# base frame for the arm
_C.ROBOT_BASE_FRAME = 'yumi_body'
_C.CLASS = 'Yumi'

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
# damped inverse kinematics value
_C.PYBULLET_IK_DAMPING = 0.0005

# NOTE: Order of joints on yumi is [1, 2, 7, 3, 4, 5, 6]
# these torques are listed in that order
# _C.MAX_TORQUES = [14, 30, 13, 14, 1, 3.5, 0.2]
_C.MAX_TORQUES = [42, 90, 39, 42, 3, 12, 1]


def get_yumi_arm_cfg():
    return _C.clone()
