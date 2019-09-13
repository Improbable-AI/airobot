from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


class Camera(object):
    """
    Base class for cameras
    """
    __metaclass__ = ABCMeta

    def __init__(self, cfgs):
        """
        Constructor for Camera base class.
        :param cfgs: configurations for camera
        :type cfgs: YACS CfgNode
        """
        self.cfgs = cfgs
