import numpy as np

from airobot.sensor.camera.camera import Camera


class PyBulletCamera(Camera):
    def __init__(self, publlet, cfgs):
        super(PyBulletCamera, self).__init__(cfgs=cfgs)
        self.p = publlet
        self.view_matrix = None
        self.proj_matrix = None

    def setup_camera(self, focus_pt=None, dist=3, yaw=0, pitch=0, roll=0):
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
        """
        if focus_pt is None:
            focus_pt = [0, 0, 0]
        if len(focus_pt) != 3:
            raise ValueError('Length of focus_pt should be 3 ([x, y, z]).')
        p = self.p
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(focus_pt,
                                                               dist,
                                                               yaw,
                                                               pitch,
                                                               roll,
                                                               upAxisIndex=2)
        height = self.cfgs.CAM_SIM.HEIGHT
        width = self.cfgs.CAM_SIM.WIDTH
        aspect = width / float(height)
        znear = self.cfgs.CAM_SIM.ZNEAR
        zfar = self.cfgs.CAM_SIM.ZFAR
        fov = self.cfgs.CAM_SIM.FOV
        self.proj_matrix = self.p.computeProjectionMatrixFOV(fov,
                                                             aspect,
                                                             znear,
                                                             zfar)

    def get_images(self, get_rgb=True, get_depth=True, **kwargs):
        """
        Return rgba/depth images

        Args:
            get_rgb (bool): return rgb image if True, None otherwise
            get_depth (bool): return depth image if True, None otherwise

        Returns:
            rgba and depth images (np.ndarray)
        """

        if self.view_matrix is None:
            raise ValueError('Please call setup_camera() first!')
        height = self.cfgs.CAM_SIM.HEIGHT
        width = self.cfgs.CAM_SIM.WIDTH
        p = self.p
        images = self.p.getCameraImage(width=width,
                                       height=height,
                                       viewMatrix=self.view_matrix,
                                       projectionMatrix=self.proj_matrix,
                                       shadow=True,
                                       flags=p.ER_NO_SEGMENTATION_MASK,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgba = None
        depth = None
        if get_rgb:
            rgba = np.reshape(images[2], (height, width, 4))  # 0 to 255
        if get_depth:
            depth_buffer = np.reshape(images[3], [width, height])
            znear = self.cfgs.CAM_SIM.ZNEAR
            zfar = self.cfgs.CAM_SIM.ZFAR
            depth = zfar * znear / (zfar - (zfar - znear) * depth_buffer)
        return rgba, depth
