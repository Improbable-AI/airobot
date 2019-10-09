from yacs.config import CfgNode as CN

_C = CN()
_C.ZNEAR = 0.01
_C.ZFAR = 10
_C.WIDTH = 640
_C.HEIGHT = 480
_C.FOV = 60


def get_sim_cam_cfg():
    return _C.clone()
