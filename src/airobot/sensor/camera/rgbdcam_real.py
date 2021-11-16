import threading
import time
from copy import deepcopy

import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

import airobot as ar
from airobot.sensor.camera.rgbdcam import RGBDCamera
from airobot.utils.common import to_rot_mat


class RGBDCameraReal(RGBDCamera):
    """
    Real RGBD camera.

    Args:
        cfgs (YACS CfgNode): configurations for the camera
        cam_name (str): camera name.

    Attributes:
        cfgs (YACS CfgNode): configurations for the end effector.
        img_height (int): height of the image.
        img_width (int): width of the image.
        cam_ext_mat (np.ndarray): extrinsic matrix (shape: :math:`[4, 4]`)
            for the camera.
        cam_int_mat (np.ndarray): intrinsic matrix (shape: :math:`[3, 3]`)
            for the camera.
        depth_scale (float): ratio of the depth image value
            to true depth value.
        depth_min (float): minimum depth value considered in 3D reconstruction.
        depth_max (float): maximum depth value considered in 3D reconstruction.
    """

    def __init__(self, cfgs, cam_name=None):
        super(RGBDCameraReal, self).__init__(cfgs=cfgs)
        self.depth_scale = self.cfgs.CAM.REAL.DEPTH_SCALE
        self.cam_int_mat = None
        self.img_height = None
        self.img_width = None
        self.cam_ext_mat = None  # extrinsic matrix T
        self.depth_min = self.cfgs.CAM.REAL.DEPTH_MIN
        self.depth_max = self.cfgs.CAM.REAL.DEPTH_MAX
        self._depth_topic = self.cfgs.CAM.REAL.ROSTOPIC_CAMERA_DEPTH
        self._rgb_topic = self.cfgs.CAM.REAL.ROSTOPIC_CAMERA_RGB
        self._cam_info_topic = self.cfgs.CAM.REAL.ROSTOPIC_CAMERA_INFO

        if cam_name is not None:
            self._depth_topic = self._rp_cam_name(self._depth_topic,
                                                  cam_name)
            self._rgb_topic = self._rp_cam_name(self._rgb_topic,
                                                cam_name)
            self._cam_info_topic = self._rp_cam_name(self._cam_info_topic,
                                                     cam_name)
        self._cv_bridge = CvBridge()
        self._cam_info_lock = threading.RLock()
        self._cam_img_lock = threading.RLock()
        self._rgb_img = None
        self._depth_img = None
        self._cam_info = None
        self._cam_P = None
        self._rgb_img_shape = None
        self._depth_img_shape = None
        rospy.Subscriber(self._cam_info_topic,
                         CameraInfo,
                         self._cam_info_callback)

        self._rgb_sub = message_filters.Subscriber(self._rgb_topic,
                                                   Image)
        self._depth_sub = message_filters.Subscriber(self._depth_topic,
                                                     Image)
        img_subs = [self._rgb_sub, self._depth_sub]
        self._sync = message_filters.ApproximateTimeSynchronizer(img_subs,
                                                                 queue_size=2,
                                                                 slop=0.2)
        self._sync.registerCallback(self._sync_callback)

        start_time = time.time()
        while True:
            if self.cam_int_mat is not None and self._rgb_img is not None:
                break
            time.sleep(0.02)
            if time.time() - start_time > 4:
                raise RuntimeError('Cannot fetch the camera info and images!')

        self._init_pers_mat()

    def _cam_info_callback(self, msg):
        self._cam_info_lock.acquire()
        if self._cam_info is None:
            self._cam_info = msg
        if self.img_height is None:
            self.img_height = int(msg.height)
        if self.img_width is None:
            self.img_width = int(msg.width)
        if self._cam_P is None:
            self._cam_P = np.array(msg.P).reshape((3, 4))
        if self.cam_int_mat is None:
            self.cam_int_mat = self._cam_P[:3, :3]
        self._cam_info_lock.release()

    def _sync_callback(self, color, depth):
        self._cam_img_lock.acquire()
        try:
            bgr_img = self._cv_bridge.imgmsg_to_cv2(color, "bgr8")
            self._rgb_img = bgr_img[:, :, ::-1]
            self._depth_img = self._cv_bridge.imgmsg_to_cv2(depth,
                                                            "passthrough")
            if self._rgb_img_shape is None:
                self._rgb_img_shape = self._rgb_img.shape
            if self._depth_img_shape is None:
                self._depth_img_shape = self._depth_img.shape
        except CvBridgeError as e:
            ar.log_error(e)
        self._cam_img_lock.release()

    def _rp_cam_name(self, topic, cam_name):
        """
        Replace the camera name in related ROS topics.

        Args:
            topic (str): ros topic name.
            cam_name (str): desired camera name.

        Returns:
            new ros topic name after the replacement
            of the camera name.
        """
        topic = topic.split('/')[1:]
        topic.insert(0, cam_name)
        return '/'.join(topic)

    def get_images(self, get_rgb=True, get_depth=True, **kwargs):
        """
        Return rgb/depth images.

        Args:
            get_rgb (bool): return rgb image if True, None otherwise.
            get_depth (bool): return depth image if True, None otherwise.

        Returns:
            2-element tuple containing

            - np.ndarray: rgb image (shape: :math:`[H, W, 3]`).
            - np.ndarray: depth image (shape: :math:`[H, W]`).
        """
        rgb_img = None
        depth_img = None
        self._cam_img_lock.acquire()
        if get_rgb:
            rgb_img = deepcopy(self._rgb_img)
        if get_depth:
            depth_img = deepcopy(self._depth_img)
        self._cam_img_lock.release()
        return rgb_img, depth_img
