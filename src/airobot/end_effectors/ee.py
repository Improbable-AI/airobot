class EndEffector(object):
    """
    Base class for end effector
    """

    def __init__(self, cfgs):
        """

        Args:
            cfgs (YACS CfgNode): configurations for the end effector
        """
        self.cfgs = cfgs

    def open(self, **kwargs):
        raise NotImplementedError

    def close(self, **kwargs):
        raise NotImplementedError
