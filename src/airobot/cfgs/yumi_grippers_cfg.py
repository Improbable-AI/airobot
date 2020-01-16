from yacs.config import CfgNode as CN

from airobot.cfgs.default_configs import get_cfg_defaults
from airobot.cfgs.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.yumi_arm import get_yumi_arm_cfg
from airobot.cfgs.yumi_parallel_jaw import get_yumi_parallel_jaw_cfg

_C = get_cfg_defaults()
# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = False

_C.ROBOT_DESCRIPTION = '/robot_description'
_C.PYBULLET_URDF = 'yumi_grippers.urdf'

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.ARM = get_yumi_arm_cfg()
_C.ARM.CLASS = 'Yumi'
_C.ARM.PYBULLET_IK_DAMPING = 0.0005
_C.ARM.RIGHT = CN()
_C.ARM.RIGHT.HAS_EETOOL = True
_C.ARM.RIGHT.HAS_CAMERA = _C.HAS_CAMERA
_C.ARM.RIGHT.HAS_ARM = _C.HAS_ARM

_C.ARM.RIGHT.ARM = get_yumi_arm_cfg()
_C.ARM.RIGHT.ARM.PYBULLET_IK_DAMPING = 0.0005
_C.ARM.RIGHT.ARM.NAME = 'right'
_C.ARM.RIGHT.ARM.JOINT_NAMES = [
    'yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r',
    'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r',
    'yumi_joint_6_r'
]

_C.ARM.RIGHT.ARM.MAX_TORQUES = [300, 300, 300, 300, 300, 300, 300]
_C.ARM.RIGHT.ARM.ROBOT_EE_FRAME = 'yumi_link_7_r'
_C.ARM.RIGHT.ARM.ROBOT_EE_FRAME_JOINT = 'yumi_joint_6_r'
_C.ARM.RIGHT.ARM.HOME_POSITION = [
    0.413, -1.325, -1.040, -0.053, -0.484, 0.841, -1.546]

_C.ARM.LEFT = CN()
_C.ARM.LEFT.HAS_EETOOL = True
_C.ARM.LEFT.HAS_CAMERA = _C.HAS_CAMERA
_C.ARM.LEFT.HAS_ARM = _C.HAS_ARM

_C.ARM.LEFT.ARM = get_yumi_arm_cfg()
_C.ARM.LEFT.ARM.PYBULLET_IK_DAMPING = 0.0005
_C.ARM.LEFT.ARM.NAME = 'left'
_C.ARM.LEFT.ARM.JOINT_NAMES = [
    'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l',
    'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l',
    'yumi_joint_6_l'
]

_C.ARM.LEFT.ARM.MAX_TORQUES = [300, 300, 300, 300, 300, 300, 300]
_C.ARM.LEFT.ARM.ROBOT_EE_FRAME = 'yumi_link_7_l'
_C.ARM.LEFT.ARM.ROBOT_EE_FRAME_JOINT = 'yumi_joint_6_l'
_C.ARM.LEFT.ARM.HOME_POSITION = [
    -0.473, -1.450, 1.091, 0.031, 0.513, 0.77, -1.669]

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.REAL = get_realsense_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'

_C.ARM.RIGHT.EETOOL = get_yumi_parallel_jaw_cfg()
_C.ARM.RIGHT.EETOOL.CLASS = 'YumiParallelJawPybullet'
_C.ARM.RIGHT.EETOOL.JOINT_NAMES = ['gripper_r_joint', 'gripper_r_joint_m']

_C.ARM.LEFT.EETOOL = get_yumi_parallel_jaw_cfg()
_C.ARM.LEFT.EETOOL.CLASS = 'YumiParallelJawPybullet'
_C.ARM.LEFT.EETOOL.JOINT_NAMES = ['gripper_l_joint', 'gripper_l_joint_m']


def get_cfg():
    return _C.clone()
