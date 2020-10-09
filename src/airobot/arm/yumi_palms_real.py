"""
Interface to ABB Yumi robot.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

from airobot.arm.yumi_real import YumiReal


class YumiPalmsReal(YumiReal):
    """
    Class interfacing with real ABB Yumi robot though ROS

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        moveit_planner (str): motion planning algorithm.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.
        wrist_cam (bool): whether the robot has a wrist camera
            mounted. If so, a box will be placed around the camera
            so that moveit is aware of the wrist camera when it's
            doing motion planning.

    Attributes:
        right_arm (YumiArmReal): Right arm instance
        left_arm (YumiArmReal): Left arm instance
    """

    def __init__(self, cfgs,
                 moveit_planner='RRTstarkConfigDefault',
                 eetool_cfg=None,
                 wrist_cam=True):
        super(YumiPalmsReal, self).__init__(cfgs=cfgs, eetool_cfg=eetool_cfg)