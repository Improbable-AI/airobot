from __future__ import absolute_import
from __future__ import division
from __future__ import print_function


class Camera(object):
    """
    Base class for cameras.

    Args:
        cfgs (YACS CfgNode): configurations for the camera.

    Attributes:
        cfgs (YACS CfgNode): configurations for the camera.
    """

    def __init__(self, cfgs):
        self.cfgs = cfgs

    def get_images(self, get_rgb=True, get_depth=True, **kwargs):
        """
        Return rgb/depth images.

        Args:
            get_rgb (bool): return rgb image if True, None otherwise.
            get_depth (bool): return depth image if True, None otherwise.

        Returns:
            2-element tuple containing

            - np.ndarray: rgb image.
            - np.ndarray: depth image.
        """
        raise NotImplementedError
