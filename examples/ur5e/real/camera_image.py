from __future__ import print_function

import cv2
import rospy

from airobot import Robot

if __name__ == "__main__":
    """
    This script shows how to get RGB and depth images from AIRobot.
    """
    robot = Robot('ur5e_2f140',
                  pb=False,
                  use_arm=False,
                  use_cam=True)
    while not rospy.is_shutdown():
        rgb, depth = robot.cam.get_images(get_rgb=True, get_depth=True)
        cv2.imshow('Color', rgb[:, :, ::-1])
        cv2.waitKey(10)
