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
        Constructor for Robot base class.

        :param cfgs: configurations for robot
        :type cfgs: dict
        """
        self.cfgs = cfgs
