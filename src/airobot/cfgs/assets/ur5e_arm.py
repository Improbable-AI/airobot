from yacs.config import CfgNode as CN

_C = CN()

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.CLASS = 'UR5e'
_C.MOVEGROUP_NAME = 'manipulator'
_C.ROSTOPIC_JOINT_STATES = '/joint_states'

# https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
_C.MAX_TORQUES = [150, 150, 150, 28, 28, 28]
_C.JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
]
# base frame for the arm
_C.ROBOT_BASE_FRAME = 'base'
# end-effector frame of the arm
_C.ROBOT_EE_FRAME = 'ee_tip'
_C.ROBOT_EE_FRAME_JOINT = 'ee_tip_joint'
_C.JOINT_SPEED_TOPIC = '/joint_speed'
_C.URSCRIPT_TOPIC = '/ur_driver/URScript'
# inverse kinematics position tolerance (m)
_C.IK_POSITION_TOLERANCE = 0.01
# inverse kinematics orientation tolerance (rad)
_C.IK_ORIENTATION_TOLERANCE = 0.05
_C.HOME_POSITION = [0, -1.66, -1.92, -1.12, 1.57, 0]
_C.MAX_JOINT_ERROR = 0.01
_C.MAX_JOINT_VEL_ERROR = 0.05
_C.MAX_EE_POS_ERROR = 0.01
# real part of the quaternion difference should be
# greater than 1-error
_C.MAX_EE_ORI_ERROR = 0.02
_C.TIMEOUT_LIMIT = 10

# reset position for the robot in pybullet
_C.PYBULLET_RESET_POS = [0, 0, 1]
# reset orientation (euler angles) for the robot in pybullet
_C.PYBULLET_RESET_ORI = [0, 0, 0]
_C.PYBULLET_IK_DAMPING = 0.0005


def get_ur5e_arm_cfg():
    return _C.clone()
