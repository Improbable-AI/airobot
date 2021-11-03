import numpy as np

from airobot.sensor.camera.camera import Camera
from airobot.utils.common import to_rot_mat


class RGBDCamera(Camera):
    """
    A RGBD camera.

    Args:
        cfgs (YACS CfgNode): configurations for the camera.

    Attributes:
        cfgs (YACS CfgNode): configurations for the end effector.
        img_height (int): height of the image.
        img_width (int): width of the image.
        cam_ext_mat (np.ndarray): extrinsic matrix (shape: :math:`[4, 4]`)
            for the camera (source frame: base frame.
            target frame: camera frame).
        cam_int_mat (np.ndarray): intrinsic matrix (shape: :math:`[3, 3]`)
            for the camera.
        cam_int_mat_inv (np.ndarray): inverse of the intrinsic matrix.
        depth_scale (float): ratio of the depth image value
            to true depth value.
        depth_min (float): minimum depth value considered in 3D reconstruction.
        depth_max (float): maximum depth value considered in 3D reconstruction.
    """

    def __init__(self, cfgs):
        super(RGBDCamera, self).__init__(cfgs=cfgs)
        self.img_height = None
        self.img_width = None
        self.cam_ext_mat = None
        self.cam_int_mat = None
        self.cam_int_mat_inv = None
        self.depth_scale = None
        self.depth_min = None
        self.depth_max = None

    def _init_pers_mat(self):
        """
        Initialize related matrices for projecting
        pixels to points in camera frame.
        """
        self.cam_int_mat_inv = np.linalg.inv(self.cam_int_mat)

        img_pixs = np.mgrid[0: self.img_height,
                            0: self.img_width].reshape(2, -1)
        img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
        self._uv_one = np.concatenate((img_pixs,
                                       np.ones((1, img_pixs.shape[1]))))
        self._uv_one_in_cam = np.dot(self.cam_int_mat_inv, self._uv_one)

    def get_cam_ext(self):
        """
        Return the camera's extrinsic matrix.

        Returns:
            np.ndarray: extrinsic matrix (shape: :math:`[4, 4]`)
            for the camera (source frame: base frame.
            target frame: camera frame).
        """
        return self.cam_ext_mat

    def get_cam_int(self):
        """
        Return the camera's intrinsic matrix.

        Returns:
            np.ndarray: intrinsic matrix (shape: :math:`[3, 3]`)
            for the camera.
        """
        return self.cam_int_mat

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
        if cam_ext is not None:
            self.cam_ext_mat = cam_ext
        else:
            if pos is None or ori is None:
                raise ValueError('If cam_ext is not provided, '
                                 'both pos and ori need '
                                 'to be provided.')
            ori = to_rot_mat(ori)
            cam_mat = np.eye(4)
            cam_mat[:3, :3] = ori
            cam_mat[:3, 3] = pos.flatten()
            self.cam_ext_mat = cam_mat

    def get_pix_3dpt(self, rs, cs, in_world=True, filter_depth=False,
                     k=1, ktype='median', depth_min=None, depth_max=None,
                     cam_ext_mat=None):
        """
        Calculate the 3D position of pixels in the RGB image.

        Args:
            rs (int or list or np.ndarray): rows of interest.
                It can be a list or 1D numpy array
                which contains the row indices. The default value is None,
                which means all rows.
            cs (int or list or np.ndarray): columns of interest.
                It can be a list or 1D numpy array
                which contains the column indices. The default value is None,
                which means all columns.
            in_world (bool): if True, return the 3D position in
                the world frame,
                Otherwise, return the 3D position in the camera frame.
            filter_depth (bool): if True, only pixels with depth values
                between [depth_min, depth_max]
                will remain.
            k (int): kernel size. A kernel (slicing window) will be used
               to get the neighboring depth values of the pixels specified
               by rs and cs. And depending on the ktype, a corresponding
               method will be applied to use some statistical value
               (such as minimum, maximum, median, mean) of all the depth
               values in the slicing window as a more robust estimate of
               the depth value of the specified pixels.
            ktype (str): what kind of statistical value of all the depth
               values in the sliced kernel
               to use as a proxy of the depth value at specified pixels.
               It can be `median`, `min`, `max`, `mean`.
            depth_min (float): minimum depth value. If None, it will use the
                default minimum depth value defined in the config file.
            depth_max (float): maximum depth value. If None, it will use the
                default maximum depth value defined in the config file.
            cam_ext_mat (np.ndarray): camera extrinsic matrix (shape: :math:`[4,4]`).
                If provided, it will be used to compute the points in the world frame.

        Returns:
            np.ndarray: 3D point coordinates of the pixels in
            camera frame (shape: :math:`[N, 3]`).
        """
        if not isinstance(rs, int) and not isinstance(rs, list) and \
                not isinstance(rs, np.ndarray):
            raise TypeError('rs should be an int, a list or a numpy array')
        if not isinstance(cs, int) and not isinstance(cs, list) and \
                not isinstance(cs, np.ndarray):
            raise TypeError('cs should be an int, a list or a numpy array')
        if isinstance(rs, int):
            rs = [rs]
        if isinstance(cs, int):
            cs = [cs]
        if isinstance(rs, np.ndarray):
            rs = rs.flatten()
        if isinstance(cs, np.ndarray):
            cs = cs.flatten()
        if not (isinstance(k, int) and (k % 2) == 1):
            raise TypeError('k should be a positive odd integer.')
        _, depth_im = self.get_images(get_rgb=False, get_depth=True)
        if k == 1:
            depth_im = depth_im[rs, cs]
        else:
            depth_im_list = []
            if ktype == 'min':
                ktype_func = np.min
            elif ktype == 'max':
                ktype_func = np.max
            elif ktype == 'median':
                ktype_func = np.median
            elif ktype == 'mean':
                ktype_func = np.mean
            else:
                raise TypeError('Unsupported ktype:[%s]' % ktype)
            for r, c in zip(rs, cs):
                s = k // 2
                rmin = max(0, r - s)
                rmax = min(self.img_height, r + s + 1)
                cmin = max(0, c - s)
                cmax = min(self.img_width, c + s + 1)
                depth_im_list.append(ktype_func(depth_im[rmin:rmax,
                                                cmin:cmax]))
            depth_im = np.array(depth_im_list)

        depth = depth_im.reshape(-1) * self.depth_scale
        img_pixs = np.stack((rs, cs)).reshape(2, -1)
        img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
        depth_min = depth_min if depth_min else self.depth_min
        depth_max = depth_max if depth_max else self.depth_max
        if filter_depth:
            valid = depth > depth_min
            valid = np.logical_and(valid,
                                   depth < depth_max)
            depth = depth[:, valid]
            img_pixs = img_pixs[:, valid]
        uv_one = np.concatenate((img_pixs,
                                 np.ones((1, img_pixs.shape[1]))))
        uv_one_in_cam = np.dot(self.cam_int_mat_inv, uv_one)
        pts_in_cam = np.multiply(uv_one_in_cam, depth)
        if in_world:
            if self.cam_ext_mat is None and cam_ext_mat is None:
                raise ValueError('Please call set_cam_ext() first to set up'
                                 ' the camera extrinsic matrix')
            cam_ext_mat = self.cam_ext_mat if cam_ext_mat is None else cam_ext_mat
            pts_in_cam = np.concatenate((pts_in_cam,
                                         np.ones((1, pts_in_cam.shape[1]))),
                                        axis=0)
            pts_in_world = np.dot(cam_ext_mat, pts_in_cam)
            pts_in_world = pts_in_world[:3, :].T
            return pts_in_world
        else:
            return pts_in_cam.T

    def get_pcd(self, in_world=True, filter_depth=True,
                depth_min=None, depth_max=None, cam_ext_mat=None,
                rgb_image=None, depth_image=None):
        """
        Get the point cloud from the entire depth image
        in the camera frame or in the world frame.

        Args:
            in_world (bool): return point cloud in the world frame, otherwise,
                return point cloud in the camera frame.
            filter_depth (bool): only return the point cloud with depth values
                lying in [depth_min, depth_max].
            depth_min (float): minimum depth value. If None, it will use the
                default minimum depth value defined in the config file.
            depth_max (float): maximum depth value. If None, it will use the
                default maximum depth value defined in the config file.
            cam_ext_mat (np.ndarray): camera extrinsic matrix (shape: :math:`[4,4]`).
                If provided, it will be used to compute the points in the world frame.
            rgb_image (np.ndarray): externally captured RGB image, if we want to
                convert a depth image captured outside this function to a point cloud.
                (shape :math:`[H, W, 3]`)
            depth_image (np.ndarray): externally captured depth image, if we want to
                convert a depth image captured outside this function to a point cloud.
                (shape :math:`[H, W]`)

        Returns:
            2-element tuple containing

            - np.ndarray: point coordinates (shape: :math:`[N, 3]`).
            - np.ndarray: rgb values (shape: :math:`[N, 3]`).
        """
        if depth_image is None or rgb_image is None:
            rgb_im, depth_im = self.get_images(get_rgb=True, get_depth=True)
        else:
            rgb_im = rgb_image
            depth_im = depth_image
        # pcd in camera from depth
        depth = depth_im.reshape(-1) * self.depth_scale
        rgb = None
        if rgb_im is not None:
            rgb = rgb_im.reshape(-1, 3)
        depth_min = depth_min if depth_min else self.depth_min
        depth_max = depth_max if depth_max else self.depth_max
        if filter_depth:
            valid = depth > depth_min
            valid = np.logical_and(valid,
                                   depth < depth_max)
            depth = depth[valid]
            if rgb is not None:
                rgb = rgb[valid]
            uv_one_in_cam = self._uv_one_in_cam[:, valid]
        else:
            uv_one_in_cam = self._uv_one_in_cam
        pts_in_cam = np.multiply(uv_one_in_cam, depth)
        if not in_world:
            pcd_pts = pts_in_cam.T
            pcd_rgb = rgb
            return pcd_pts, pcd_rgb
        else:
            if self.cam_ext_mat is None and cam_ext_mat is None:
                raise ValueError('Please call set_cam_ext() first to set up'
                                 ' the camera extrinsic matrix')
            cam_ext_mat = self.cam_ext_mat if cam_ext_mat is None else cam_ext_mat
            pts_in_cam = np.concatenate((pts_in_cam,
                                         np.ones((1, pts_in_cam.shape[1]))),
                                        axis=0)
            pts_in_world = np.dot(cam_ext_mat, pts_in_cam)
            pcd_pts = pts_in_world[:3, :].T
            pcd_rgb = rgb
            return pcd_pts, pcd_rgb
