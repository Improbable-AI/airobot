from yacs.config import CfgNode as CN

_C = CN()
# joint angle when the gripper is fully open
_C.OPEN_ANGLE = 0.025
# joint angle when the gripper is fully closed
_C.CLOSE_ANGLE = 0.0

_C.MAX_TORQUE = 100.0

_C.CLASS = 'YumiParallelJawPybullet'


def get_yumi_parallel_jaw_cfg():
    return _C.clone()
