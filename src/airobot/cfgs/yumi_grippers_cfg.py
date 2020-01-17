from yacs.config import CfgNode as CN

from airobot.cfgs.assets.default_configs import get_cfg_defaults
from airobot.cfgs.assets.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.assets.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.assets.yumi_dual_arm import get_yumi_dual_arm_cfg
from airobot.cfgs.assets.yumi_parallel_jaw import get_yumi_parallel_jaw_cfg


_C = get_yumi_dual_arm_cfg()

_C.ROBOT_DESCRIPTION = '/robot_description'

_C.PYBULLET_URDF = 'yumi_grippers.urdf'

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.REAL = get_realsense_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'

_C.ARM.RIGHT.HAS_EETOOL = True

_C.ARM.RIGHT.EETOOL = get_yumi_parallel_jaw_cfg()
_C.ARM.RIGHT.EETOOL.JOINT_NAMES = ['gripper_r_joint', 'gripper_r_joint_m']

_C.ARM.LEFT.HAS_EETOOL = True

_C.ARM.LEFT.EETOOL = get_yumi_parallel_jaw_cfg()
_C.ARM.LEFT.EETOOL.JOINT_NAMES = ['gripper_l_joint', 'gripper_l_joint_m']


def get_cfg():
    return _C.clone()
