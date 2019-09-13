# noinspection PyPep8Naming
from yacs.config import CfgNode as CN

_C = CN()
_C.URDF = ''


def get_cfg_defaults():
    return _C.clone()
