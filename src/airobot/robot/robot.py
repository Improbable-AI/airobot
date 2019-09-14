from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from abc import ABCMeta


class Robot(object):
    """
    Base class for robots
    """
    __metaclass__ = ABCMeta

    def __init__(self, cfgs):
        """

        Args:
            cfgs (YACS CfgNode): configurations for the robot
        """
        self.cfgs = cfgs

    def go_home(self):
        raise NotImplementedError

    def set_jpos(self, position, joint_name=None, *args, **kwargs):
        raise NotImplementedError

    def set_jvel(self, velocity, joint_name=None, *args, **kwargs):
        raise NotImplementedError

    def set_jtorq(self, torque, joint_name=None, *args, **kwargs):
        raise NotImplementedError

    def set_ee_pose(self, pos, ori=None, *args, **kwargs):
        raise NotImplementedError

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, *args, **kwargs):
        raise NotImplementedError

    def get_jpos(self, joint_name=None):
        raise NotImplementedError

    def get_jvel(self, joint_name=None):
        raise NotImplementedError

    def get_jtorq(self, joint_name=None):
        raise NotImplementedError

    def get_ee_pose(self):
        raise NotImplementedError

    def compute_ik(self, pos, ori=None):
        raise NotImplementedError
