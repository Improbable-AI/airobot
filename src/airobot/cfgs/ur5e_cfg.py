from yacs.config import CfgNode as CN

from airobot.cfgs.arm import get_cfg_defaults
from airobot.cfgs.pybullet_camera import get_sim_cam_cfg
from airobot.cfgs.realsense_camera import get_realsense_cam_cfg

_C = get_cfg_defaults()

_C.URDF = 'ur5e_2f140_pybullet.urdf'
_C.MAX_JOINT_ERROR = 0.01
_C.TIMEOUT_LIMIT = 5
_C.CAM_SIM = get_sim_cam_cfg()
_C.CAM_REALSENSE = get_realsense_cam_cfg()
_C.CAM_KINECT = CN()


def get_cfg():
    return _C.clone()
