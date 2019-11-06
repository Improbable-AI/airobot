from airobot.cfgs.default_configs import get_cfg_defaults
from airobot.cfgs.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.robotiq2f140 import get_robotiq2f140_cfg

_C = get_cfg_defaults()
# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = True

_C.ROBOT_DESCRIPTION = '/robot_description'
_C.PYBULLET_URDF = 'ur5e_2f140_pybullet.urdf'

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.ARM.CLASS = 'Yumi'
_C.ARM.MOVEGROUP_NAME = 'manipulator'
_C.ARM.ROSTOPIC_JOINT_STATES = '/joint_states'

_C.RIGHT.ARM.JOINT_NAMES = [
    'yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r',
    'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r',
    'yumi_joint_7_r'
]
_C.RIGHT.ARM.MAX_TORQUES = [150, 150, 150, 30, 30, 30, 30]
_C.RIGHT.ARM.ROBOT_EE_FRAME = 'yumi_link_7_r'
_C.RIGHT.ARM.ROBOT_EE_FRAME_JOINT = 'yumi_joint_6_r'


_C.LEFT.ARM.JOINT_NAMES = [
    'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l',
    'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l',
    'yumi_joint_7_l'
]
_C.LEFT.ARM.MAX_TORQUES = [150, 150, 150, 30, 30, 30, 30]
_C.LEFT.ARM.ROBOT_EE_FRAME = 'yumi_link_7_l'
_C.LEFT.ARM.ROBOT_EE_FRAME_JOINT = 'yumi_joint_6_l'

# base frame for the arm
_C.ARM.ROBOT_BASE_FRAME = 'yumi_body'
# end-effector frame of the arm

_C.ARM.JOINT_SPEED_TOPIC = '/joint_speed'

# inverse kinematics position tolerance (m)
_C.ARM.IK_POSITION_TOLERANCE = 0.01
# inverse kinematics orientation tolerance (rad)
_C.ARM.IK_ORIENTATION_TOLERANCE = 0.1
_C.ARM.HOME_POSITION = [0, -1.66, -1.92, -1.12, 1.57, 0]
_C.ARM.MAX_JOINT_ERROR = 0.01
_C.ARM.MAX_JOINT_VEL_ERROR = 0.1
_C.ARM.MAX_EE_POS_ERROR = 0.01
# real part of the quaternion difference should be
# greater than 1-error
_C.ARM.MAX_EE_ORI_ERROR = 0.02
_C.ARM.TIMEOUT_LIMIT = 10

# reset position for the robot in pybullet
_C.ARM.PYBULLET_RESET_POS = [0, 0, 1]
# reset orientation (euler angles) for the robot in pybullet
_C.ARM.PYBULLET_RESET_ORI = [0, 0, 0]

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.REAL = get_realsense_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'

_C.EETOOL = get_robotiq2f140_cfg()
_C.EETOOL.CLASS = 'Robotiq2F140'


def get_cfg():
    return _C.clone()
