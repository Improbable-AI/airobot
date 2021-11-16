import numpy as np

from airobot.sensor.camera.rgbdcam import RGBDCamera
from airobot.utils.common import create_se3
from airobot.utils.common import rotvec2rot


class RGBDCameraPybullet(RGBDCamera):
    """
    RGBD Camera in Pybullet.

    Args:
        cfgs (YACS CfgNode): configurations for the camera.

    Attributes:
        view_matrix (np.ndarray): view matrix of opengl
            camera (shape: :math:`[4, 4]`).
        proj_matrix (np.ndarray): projection matrix of
            opengl camera (shape: :math:`[4, 4]`).
    """

    def __init__(self, cfgs, pb_client):
        self._pb = pb_client
        super(RGBDCameraPybullet, self).__init__(cfgs=cfgs)
        self.view_matrix = None
        self.proj_matrix = None
        self.depth_scale = 1
        self.depth_min = self.cfgs.CAM.SIM.ZNEAR
        self.depth_max = self.cfgs.CAM.SIM.ZFAR
        self._set_cam_int()

    def _set_cam_int(self, img_height=None, img_width=None):
        self.img_height = img_height if img_height else self.cfgs.CAM.SIM.HEIGHT
        self.img_width = img_width if img_width else self.cfgs.CAM.SIM.WIDTH
        aspect = self.img_width / float(self.img_height)
        znear = self.cfgs.CAM.SIM.ZNEAR
        zfar = self.cfgs.CAM.SIM.ZFAR
        fov = self.cfgs.CAM.SIM.FOV
        pm = self._pb.computeProjectionMatrixFOV(fov,
                                                 aspect,
                                                 znear,
                                                 zfar)
        self.proj_matrix = np.array(pm).reshape(4, 4)
        vfov = np.deg2rad(fov)
        tan_half_vfov = np.tan(vfov / 2.0)
        tan_half_hfov = tan_half_vfov * self.img_width / float(self.img_height)
        # focal length in pixel space
        fx = self.img_width / 2.0 / tan_half_hfov
        fy = self.img_height / 2.0 / tan_half_vfov
        self.cam_int_mat = np.array([[fx, 0, self.img_width / 2.0],
                                     [0, fy, self.img_height / 2.0],
                                     [0, 0, 1]])
        self._init_pers_mat()

    def setup_camera(self, focus_pt=None, dist=3,
                     yaw=0, pitch=0, roll=0,
                     height=None, width=None):
        """
        Setup the camera view matrix and projection matrix. Must be called
        first before images are renderred.

        Args:
            focus_pt (list): position of the target (focus) point,
                in Cartesian world coordinates.
            dist (float): distance from eye (camera) to the focus point.
            yaw (float): yaw angle in degrees,
                left/right around up-axis (z-axis).
            pitch (float): pitch in degrees, up/down.
            roll (float): roll in degrees around forward vector.
            height (float): height of image. If None, it will use
                the default height from the config file.
            width (float): width of image. If None, it will use
                the default width from the config file.
        """
        if focus_pt is None:
            focus_pt = [0, 0, 0]
        if len(focus_pt) != 3:
            raise ValueError('Length of focus_pt should be 3 ([x, y, z]).')
        vm = self._pb.computeViewMatrixFromYawPitchRoll(focus_pt,
                                                        dist,
                                                        yaw,
                                                        pitch,
                                                        roll,
                                                        upAxisIndex=2)
        self.view_matrix = np.array(vm).reshape(4, 4)

        rot = create_se3(rotvec2rot(np.pi * np.array([1, 0, 0])))
        view_matrix_T = self.view_matrix.T
        self.cam_ext_mat = np.dot(np.linalg.inv(view_matrix_T), rot)

        if height is not None or width is not None:
            self._set_cam_int(img_height=height, img_width=width)

    def set_cam_ext(self, pos=None, ori=None, cam_ext=None):
        """
        Set the camera extrinsic matrix.

        Args:
            pos (np.ndarray): position of the camera (shape: :math:`[3,]`).
            ori (np.ndarray): orientation.
                It can be rotation matrix (shape: :math:`[3, 3]`)
                quaternion ([x, y, z, w], shape: :math:`[4]`), or
                euler angles ([roll, pitch, yaw], shape: :math:`[3]`).
            cam_ext (np.ndarray): extrinsic matrix (shape: :math:`[4, 4]`).
                If this is provided, pos and ori will be ignored.
        """
        super().set_cam_ext(pos=pos, ori=ori, cam_ext=cam_ext)
        transform = np.eye(4)
        transform[:3, :3] = rotvec2rot(np.pi * np.array([1, 0, 0]))
        view_matrix = np.linalg.inv(np.dot(self.cam_ext_mat, transform)).T
        self.view_matrix = view_matrix

    def get_images(self, get_rgb=True, get_depth=True,
                   get_seg=False, **kwargs):
        """
        Return rgb, depth, and segmentation images.

        Args:
            get_rgb (bool): return rgb image if True, None otherwise.
            get_depth (bool): return depth image if True, None otherwise.
            get_seg (bool): return the segmentation mask if True,
                None otherwise.

        Returns:
            2-element tuple (if `get_seg` is False) containing

            - np.ndarray: rgb image (shape: [H, W, 3]).
            - np.ndarray: depth image (shape: [H, W]).

            3-element tuple (if `get_seg` is True) containing

            - np.ndarray: rgb image (shape: [H, W, 3]).
            - np.ndarray: depth image (shape: [H, W]).
            - np.ndarray: segmentation mask image (shape: [H, W]), with
              pixel values corresponding to object id and link id.
              From the PyBullet documentation, the pixel value
              "combines the object unique id and link index as follows:
              value = objectUniqueId + (linkIndex+1)<<24 ...
              for a free floating body without joints/links, the
              segmentation mask is equal to its body unique id,
              since its link index is -1.".
        """

        if self.view_matrix is None:
            raise ValueError('Please call setup_camera() first!')
        if self._pb.opengl_render:
            renderer = self._pb.ER_BULLET_HARDWARE_OPENGL
        else:
            renderer = self._pb.ER_TINY_RENDERER
        cam_img_kwargs = {
            'width': self.img_width,
            'height': self.img_height,
            'viewMatrix': self.view_matrix.flatten(),
            'projectionMatrix': self.proj_matrix.flatten(),
            'flags': self._pb.ER_NO_SEGMENTATION_MASK,
            'renderer': renderer
        }
        if get_seg:
            pb_seg_flag = self._pb.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
            cam_img_kwargs['flags'] = pb_seg_flag

        cam_img_kwargs.update(kwargs)
        images = self._pb.getCameraImage(**cam_img_kwargs)
        rgb = None
        depth = None
        if get_rgb:
            rgb = np.reshape(images[2],
                             (self.img_height,
                              self.img_width, 4))[:, :, :3]  # 0 to 255
        if get_depth:
            depth_buffer = np.reshape(images[3], [self.img_height,
                                                  self.img_width])
            znear = self.cfgs.CAM.SIM.ZNEAR
            zfar = self.cfgs.CAM.SIM.ZFAR
            depth = zfar * znear / (zfar - (zfar - znear) * depth_buffer)
        if get_seg:
            seg = np.reshape(images[4], [self.img_height,
                                         self.img_width])
            return rgb, depth, seg
        else:
            return rgb, depth
