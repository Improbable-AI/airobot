from yacs.config import CfgNode as CN

_C = CN()

# whether the robot has an arm or not
_C.HAS_ARM = False
# whether the robot has a mobile base or not
_C.HAS_BASE = False
# whether the robot has a camera or not
_C.HAS_CAMERA = False
# whether the robot has a end effector tool or not
_C.HAS_EETOOL = False

_C.ARM = CN()
_C.ARM.CLASS = ''

_C.CAM = CN()
_C.CAM.CLASS = ''

_C.BASE = CN()
_C.BASE.CLASS = ''

_C.EETOOL = CN()
_C.EETOOL.CLASS = ''


def get_cfg_defaults():
    return _C.clone()
