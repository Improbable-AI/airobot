import threading
import time
from copy import deepcopy

import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf import TransformListener

import airobot as ar
from airobot.sensor.camera.rgbdcam import RGBDCamera
from airobot.utils.common import to_rot_mat


class RGBDCameraReal(RGBDCamera):
    def __init__(self, cfgs, cam_name=None):
        """
        Initialize the rgbd camera

        Args:
            cfgs (YACS CfgNode): configurations for the camera
            cam_name (str): camera name
        """
        super(RGBDCameraReal, self).__init__(cfgs=cfgs)
        self.depth_topic = self.cfgs.CAM.REAL.ROSTOPIC_CAMERA_DEPTH
        self.rgb_topic = self.cfgs.CAM.REAL.ROSTOPIC_CAMERA_RGB
        self.cam_info_topic = self.cfgs.CAM.REAL.ROSTOPIC_CAMERA_INFO
        self.depth_scale = self.cfgs.CAM.REAL.DEPTH_SCALE
        if cam_name is not None:
            self.depth_topic = self._rp_cam_name(self.depth_topic,
                                                 cam_name)
            self.rgb_topic = self._rp_cam_name(self.rgb_topic,
                                               cam_name)
            self.cam_info_topic = self._rp_cam_name(self.cam_info_topic,
                                                    cam_name)
        self.cv_bridge = CvBridge()
        self.cam_info_lock = threading.RLock()
        self.cam_img_lock = threading.RLock()
        self._tf_listener = TransformListener()
        self.rgb_img = None
        self.depth_img = None
        self.cam_info = None
        self.cam_P = None
        self.cam_int_mat = None
        self.cam_height = None
        self.cam_width = None
        self.cam_ext_mat = None  # extrinsic matrix T
        self.rgb_img_shape = None
        self.depth_img_shape = None
        rospy.Subscriber(self.cam_info_topic,
                         CameraInfo,
                         self._cam_info_callback)

        self.rgb_sub = message_filters.Subscriber(self.rgb_topic,
                                                  Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic,
                                                    Image)
        img_subs = [self.rgb_sub, self.depth_sub]
        self.sync = message_filters.ApproximateTimeSynchronizer(img_subs,
                                                                queue_size=2,
                                                                slop=0.2)
        self.sync.registerCallback(self._sync_callback)
        self.depth_min = self.cfgs.CAM.REAL.DEPTH_MIN
        self.depth_max = self.cfgs.CAM.REAL.DEPTH_MAX
        start_time = time.time()
        while True:
            if self.cam_int_mat is not None and self.rgb_img is not None:
                break
            time.sleep(0.02)
            if time.time() - start_time > 4:
                raise RuntimeError('Cannot fetch the camera info and images!')

        self._init_pers_mat()

    def _cam_info_callback(self, msg):
        self.cam_info_lock.acquire()
        if self.cam_info is None:
            self.cam_info = msg
        if self.cam_height is None:
            self.cam_height = int(msg.height)
        if self.cam_width is None:
            self.cam_width = int(msg.width)
        if self.cam_P is None:
            self.cam_P = np.array(msg.P).reshape((3, 4))
        if self.cam_int_mat is None:
            self.cam_int_mat = self.cam_P[:3, :3]
        self.cam_info_lock.release()

    def _sync_callback(self, color, depth):
        self.cam_img_lock.acquire()
        try:
            bgr_img = self.cv_bridge.imgmsg_to_cv2(color, "bgr8")
            self.rgb_img = bgr_img[:, :, ::-1]
            self.depth_img = self.cv_bridge.imgmsg_to_cv2(depth,
                                                          "passthrough")
            if self.rgb_img_shape is None:
                self.rgb_img_shape = self.rgb_img.shape
            if self.depth_img_shape is None:
                self.depth_img_shape = self.depth_img.shape
        except CvBridgeError as e:
            ar.log_error(e)
        self.cam_img_lock.release()

    def _rp_cam_name(self, topic, cam_name):
        """
        Replace the camera name in related ROS topics

        Args:
            topic (str): ros topic name
            cam_name (str): desired camera name

        Returns:
            new ros topic name after the replacement
            of the camera name
        """
        topic = topic.split('/')[1:]
        topic.insert(0, cam_name)
        return '/'.join(topic)

    def set_cam_ext(self, pos=None, ori=None, cam_ext=None):
        """
        Set the camera extrinsic matrix

        Args:
            pos (np.ndarray): position of the camera (shape: [3,])
            ori (np.ndarray): orientation.
                It can be rotation matrix (shape:[3, 3])
                quaternion ([x, y, z, w], shape: [4]), or
                euler angles ([roll, pitch, yaw], shape: [3])
            cam_ext (np.ndarray): extrinsic matrix (shape: [4, 4]).
                If this is provided, pos and ori will be ignored
        """
        if cam_ext is not None:
            self.cam_ext_mat = cam_ext
        else:
            if pos is None or ori is None:
                raise ValueError('If cam_ext is not provided, '
                                 'both pos and ori need'
                                 'to be provided.')
            ori = to_rot_mat(ori)
            cam_mat = np.eye(4)
            cam_mat[:3, :3] = ori
            cam_mat[:3, 3] = pos.flatten()
            self.cam_ext_mat = cam_mat

    def get_images(self, get_rgb=True, get_depth=True, **kwargs):
        """
        Return rgb/depth images

        Args:
            get_rgb (bool): return rgb image if True, None otherwise
            get_depth (bool): return depth image if True, None otherwise

        Returns:
            2-element tuple containing

            - np.ndarray: rgb image (shape: [H, W, 3])
            - np.ndarray: depth image (shape: [H, W])
        """
        rgb_img = None
        depth_img = None
        self.cam_img_lock.acquire()
        if get_rgb:
            rgb_img = deepcopy(self.rgb_img)
        if get_depth:
            depth_img = deepcopy(self.depth_img)
        self.cam_img_lock.release()
        return rgb_img, depth_img
