from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from abc import ABCMeta


class Camera(object):
    """
    Base class for cameras
    """
    __metaclass__ = ABCMeta

    def __init__(self, cfgs):
        """

        Args:
            cfgs (YACS CfgNode): configurations for the camera
        """
        self.cfgs = cfgs
