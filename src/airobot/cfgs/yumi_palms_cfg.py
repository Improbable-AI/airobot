from yacs.config import CfgNode as CN

from airobot.cfgs.assets.default_configs import get_cfg_defaults
from airobot.cfgs.assets.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.assets.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.assets.yumi_dual_arm import get_yumi_dual_arm_cfg


_C = get_yumi_dual_arm_cfg()

_C.ROBOT_DESCRIPTION = '/robot_description'
_C.PYBULLET_URDF = 'yumi_gelslim_palm.urdf'

# prefix of the class name of the ARM
# if it's for pybullet simulation, the name will
# be augemented to be '<Prefix>Pybullet'
# if it's for the real robot, the name will be
# augmented to be '<Prefix>Real'
_C.ARM.CLASS = 'YumiPalms'

# yumi with palms has compliant joints at the wrist and in the gel
_C.ARM.RIGHT.ARM.COMPLIANT_JOINT_NAMES = ['yumi_palm_r', 'yumi_gel_r']
_C.ARM.RIGHT.ARM.COMPLIANT_GAINS = [5, 5]
_C.ARM.RIGHT.ARM.COMPLIANT_MAX_FORCE = 20

# yumi with palms has compliant joints at the wrist and in the gel
_C.ARM.LEFT.ARM.COMPLIANT_JOINT_NAMES = ['yumi_palm_l', 'yumi_gel_l']
_C.ARM.LEFT.ARM.COMPLIANT_GAINS = [5, 5]
_C.ARM.LEFT.ARM.COMPLIANT_MAX_FORCE = 20

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.REAL = get_realsense_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'


def get_cfg():
    return _C.clone()
