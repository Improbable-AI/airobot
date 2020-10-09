from airobot.cfgs.assets.default_configs import get_cfg_defaults
from airobot.cfgs.assets.yumi_ros_arm import get_yumi_ros_cfg
from yacs.config import CfgNode as CN

_C = get_cfg_defaults()
# whether the robot has an arm or not
_C.HAS_ARM = True
# whether the robot has a camera or not
_C.HAS_CAMERA = True
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = False

_C.ROBOT_DESCRIPTION = '/robot_description'

_C.ARM.RIGHT = CN()
_C.ARM.LEFT = CN()

_C.ARM.RIGHT.HAS_EETOOL = _C.HAS_EETOOL
_C.ARM.RIGHT.HAS_CAMERA = _C.HAS_CAMERA
_C.ARM.RIGHT.HAS_ARM = _C.HAS_ARM

_C.ARM.LEFT.HAS_EETOOL = _C.HAS_EETOOL
_C.ARM.LEFT.HAS_CAMERA = _C.HAS_CAMERA
_C.ARM.LEFT.HAS_ARM = _C.HAS_ARM

_C.ARM.RIGHT.ARM = get_yumi_ros_cfg()
_C.ARM.RIGHT.ARM.SERVICE_ROBOT_NAME = 'robot1'
srv_keys = _C.ARM.RIGHT.ARM.SERVICES.keys()
for key in srv_keys:
    _C.ARM.RIGHT.ARM.SERVICES[key] = \
        _C.ARM.RIGHT.ARM.SERVICE_ROBOT_NAME + '_' + \
        _C.ARM.RIGHT.ARM.SERVICES[key]
_C.ARM.RIGHT.ARM.EGM_TARGET_JOINTS_TOPIC = _C.ARM.RIGHT.ARM.SERVICE_ROBOT_NAME + '_' + _C.ARM.RIGHT.ARM.EGM_TARGET_JOINTS_TOPIC

_C.ARM.LEFT.ARM = get_yumi_ros_cfg()
_C.ARM.LEFT.ARM.SERVICE_ROBOT_NAME = 'robot2'
srv_keys = _C.ARM.LEFT.ARM.SERVICES.keys()
for key in srv_keys:
    _C.ARM.LEFT.ARM.SERVICES[key] = \
        _C.ARM.LEFT.ARM.SERVICE_ROBOT_NAME + '_' + \
        _C.ARM.LEFT.ARM.SERVICES[key]
_C.ARM.LEFT.ARM.EGM_TARGET_JOINTS_TOPIC = _C.ARM.LEFT.ARM.SERVICE_ROBOT_NAME + '_' + _C.ARM.LEFT.ARM.EGM_TARGET_JOINTS_TOPIC

_C.ARM.RIGHT.ARM.NAME = 'right'
_C.ARM.RIGHT.ARM.JOINT_NAMES = [
    'yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r',
    'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r',
    'yumi_joint_6_r'
]


_C.ARM.RIGHT.ARM.ROBOT_EE_FRAME = 'yumi_link_7_r'
_C.ARM.RIGHT.ARM.ROBOT_EE_FRAME_JOINT = 'yumi_joint_6_r'
_C.ARM.RIGHT.ARM.HOME_POSITION = [
    0.413, -1.325, -1.040, -0.053, -0.484, 0.841, -1.546]


_C.ARM.LEFT.ARM.NAME = 'left'
_C.ARM.LEFT.ARM.JOINT_NAMES = [
    'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l',
    'yumi_joint_3_l', 'yumi_joint_4_l', 'yumi_joint_5_l',
    'yumi_joint_6_l'
]

_C.ARM.LEFT.ARM.ROBOT_EE_FRAME = 'yumi_link_7_l'
_C.ARM.LEFT.ARM.ROBOT_EE_FRAME_JOINT = 'yumi_joint_6_l'
_C.ARM.LEFT.ARM.HOME_POSITION = [
    -0.473, -1.450, 1.091, 0.031, 0.513, 0.77, -1.669]