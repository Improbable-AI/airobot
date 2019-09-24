"""
A UR5e robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy

from airobot.robot.robot import Robot
from airobot.sensor.camera.rgbd_cam import RGBDCamera


class UR5eRobotReal(Robot):
    def __init__(self, cfgs):
        super(UR5eRobotReal, self).__init__(cfgs=cfgs)
        try:
            rospy.init_node('airobot', anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn('ROS node [airobot] has already been initialized')
        self.camera = RGBDCamera(cfgs=cfgs)
