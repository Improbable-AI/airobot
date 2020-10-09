from yumi_arm import get_yumi_arm_cfg

_C = get_yumi_arm_cfg()

# ros services
_C.ARM.SET_JOINTS_SRV = '_SetJoints'
_C.ARM.SET_CARTESIAN_SRV = '_SetCartesian'

_C.ARM.GET_JOINTS_SRV = '_GetJoints'
_C.ARM.GET_CARTESIAN_SRV = '_GetCartesian'

_C.ARM.SET_SPEED_SRV = '_SetMaxSpeed'
_C.ARM.EGM_MODE_SRV = '_SetEGMMode'
_C.ARM.ADD_BUFF_SRV = '_AddToJointBuffer'
_C.ARM.CLEAR_BUFF_SRV = '_ClearJointBuffer'
_C.ARM.EXEC_BUFF_SRV = '_ExecuteJointBuffer'
_C.ARM.EXEC_BUFF_SYNC_SRV = '_ExecuteSynchroJointBuffer'

_C.ARM.SERVICES = {
    'set_joints': _C.ARM.SET_JOINTS_SRV,
    'set_cartesian': _C.ARM.SET_CARTESIAN_SRV,
    'get_joints': _C.ARM.GET_JOINTS_SRV,
    'get_cartesian': _C.ARM.GET_CARTESIAN_SRV,
    'set_speed': _C.ARM.SET_SPEED_SRV,
    'egm_mode': _C.ARM.EGM_MODE_SRV,
    'add_buffer': _C.ARM.ADD_BUFF_SRV,
    'clear_buffer': _C.ARM.CLEAR_BUFF_SRV,
    'execute_buffer': _C.ARM.EXEC_BUFF_SRV,
    'execute_buffer_sync': _C.ARM.EXEC_BUFF_SYNC_SRV
}

_C.ARM.EGM_TARGET_JOINTS_TOPIC = '_TargetJoints'

_C.ARM.SERVICE_TIMEOUT_DEFAULT = 0.5

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.ARM.ROSTOPIC_JOINT_STATES = '/joint_states'

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


def get_yumi_ros_cfg():
    return _C.clone()
