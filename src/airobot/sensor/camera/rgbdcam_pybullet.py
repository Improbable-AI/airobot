import numpy as np
import pybullet as p

from airobot.sensor.camera.rgbdcam import RGBDCamera
from airobot.utils.pb_util import PB_CLIENT


class RGBDCameraPybullet(RGBDCamera):
    def __init__(self, cfgs):
        """
        Args:
            cfgs (YACS CfgNode): configurations for the camera
        """
        super(RGBDCameraPybullet, self).__init__(cfgs=cfgs)
        self.p = p
        self.view_matrix = None
        self.proj_matrix = None
        self.depth_scale = 1
        self.depth_min = self.cfgs.CAM.SIM.ZNEAR
        self.depth_max = self.cfgs.CAM.SIM.ZFAR

    def setup_camera(self, focus_pt=None, dist=3,
                     yaw=0, pitch=0, roll=0,
                     height=None, width=None):
        """
        Setup the camera view matrix and projection matrix. Must be called
        first before images are renderred

        Args:
            focus_pt (list): position of the target (focus) point,
                in Cartesian world coordinates
            dist (float): distance from eye (camera) to the focus point
            yaw (float): yaw angle in degrees,
                left/right around up-axis (z-axis).
            pitch (float): pitch in degrees, up/down.
            roll (float): roll in degrees around forward vector
            height (float): height of image. If None, it will use
                the default height from the config file
            width (float): width of image. If None, it will use
                the default width from the config file
        """
        if focus_pt is None:
            focus_pt = [0, 0, 0]
        if len(focus_pt) != 3:
            raise ValueError('Length of focus_pt should be 3 ([x, y, z]).')
        print("PB_CLIENT: " + str(PB_CLIENT))
        vm = p.computeViewMatrixFromYawPitchRoll(focus_pt,
                                                 dist,
                                                 yaw,
                                                 pitch,
                                                 roll,
                                                 upAxisIndex=2,
                                                 physicsClientId=PB_CLIENT)
        self.view_matrix = vm
        self.cam_height = height if height else self.cfgs.CAM.SIM.HEIGHT
        self.cam_width = width if width else self.cfgs.CAM.SIM.WIDTH
        aspect = self.cam_width / float(self.cam_height)
        znear = self.cfgs.CAM.SIM.ZNEAR
        zfar = self.cfgs.CAM.SIM.ZFAR
        fov = self.cfgs.CAM.SIM.FOV
        pm = p.computeProjectionMatrixFOV(fov,
                                          aspect,
                                          znear,
                                          zfar,
                                          physicsClientId=PB_CLIENT)
        self.proj_matrix = pm
        rot = np.array([[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]])
        view_matrix = np.array(self.view_matrix).reshape(4, 4).T
        self.cam_ext_mat = np.dot(np.linalg.inv(view_matrix), rot)

        vfov = np.deg2rad(fov)
        tan_half_vfov = np.tan(vfov / 2.0)
        tan_half_hfov = tan_half_vfov * self.cam_width / float(self.cam_height)
        fx = self.cam_width / 2.0 / tan_half_hfov  # focal length in pixel space
        fy = self.cam_height / 2.0 / tan_half_vfov
        self.cam_int_mat = np.array([[fx, 0, self.cam_width / 2.0],
                                     [0, fy, self.cam_height / 2.0],
                                     [0, 0, 1]])
        self._init_pers_mat()

    def get_cam_ext(self):
        return self.cam_ext_mat

    def get_cam_int(self):
        return self.cam_int_mat

    def get_images(self, get_rgb=True, get_depth=True, 
                   get_seg=False, **kwargs):
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

        if self.view_matrix is None:
            raise ValueError('Please call setup_camera() first!')
        if not get_seg:
            images = p.getCameraImage(width=self.cam_width,
                                      height=self.cam_height,
                                      viewMatrix=self.view_matrix,
                                      projectionMatrix=self.proj_matrix,
                                      shadow=True,
                                      flags=p.ER_NO_SEGMENTATION_MASK,
                                      renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                      physicsClientId=PB_CLIENT)
        else:
            images = p.getCameraImage(width=self.cam_width,
                                      height=self.cam_height,
                                      viewMatrix=self.view_matrix,
                                      projectionMatrix=self.proj_matrix,
                                      shadow=True,
                                      flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
                                      renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                      physicsClientId=PB_CLIENT)            
        rgb = None
        depth = None
        seg = None
        if get_rgb:
            rgb = np.reshape(images[2],
                             (self.cam_height,
                              self.cam_width, 4))[:, :, :3]  # 0 to 255
        if get_depth:
            depth_buffer = np.reshape(images[3], [self.cam_height,
                                                  self.cam_width])
            znear = self.cfgs.CAM.SIM.ZNEAR
            zfar = self.cfgs.CAM.SIM.ZFAR
            depth = zfar * znear / (zfar - (zfar - znear) * depth_buffer)
        if get_seg:
            seg = np.reshape(images[4], [self.cam_height,
                                         self.cam_width])
            return rgb, depth, seg
        else:
            return rgb, depth
