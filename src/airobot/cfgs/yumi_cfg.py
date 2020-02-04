from airobot.cfgs.assets.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.assets.realsense_camera import get_realsense_cam_cfg
from airobot.cfgs.assets.yumi_dual_arm import get_yumi_dual_arm_cfg

# _C = get_cfg_defaults()
_C = get_yumi_dual_arm_cfg()

_C.ROBOT_DESCRIPTION = '/robot_description'

# _C = get_yumi_dual_arm_cfg()
_C.PYBULLET_URDF = 'yumi.urdf'

_C.CAM.SIM = get_sim_cam_cfg()
_C.CAM.REAL = get_realsense_cam_cfg()
_C.CAM.CLASS = 'RGBDCamera'


def get_cfg():
    return _C.clone()
