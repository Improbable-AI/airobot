from yacs.config import CfgNode as CN

_C = CN()
# joint angle when the gripper is fully open
_C.GRIPPER_OPEN_ANGLE = 0
# joint angle when the gripper is fully closed
_C.GRIPPER_CLOSE_ANGLE = 0.7


def get_robotiq2f140_cfg():
    return _C.clone()
