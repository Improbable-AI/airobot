from yacs.config import CfgNode as CN

_C = CN()
# joint angle when the gripper is fully open
_C.OPEN_ANGLE = 0.0
# joint angle when the gripper is fully closed
_C.CLOSE_ANGLE = 0.7

# default host and port for socket used to
# communicate with the gripper through the
# UR controller
_C.SOCKET_HOST = '127.0.0.1'
_C.SOCKET_PORT = 63352
_C.SOCKET_NAME = 'gripper_socket'

_C.DEFAULT_SPEED = 255
_C.DEFAULT_FORCE = 50

_C.COMMAND_TOPIC = '/ur_driver/URScript'
_C.GAZEBO_COMMAND_TOPIC = '/gripper/gripper_cmd/goal'
_C.GAZEBO_SUBSCRIBE_TOPIC = '/joint_states'

# default maximum values for gripper state varibles
# minimum values are all 0
_C.POSITION_RANGE = 255
_C.POSITION_SCALING = (255 / 0.7)


def get_robotiq2f140_cfg():
    return _C.clone()
