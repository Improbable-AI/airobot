class EndEffectorTool(object):
    """
    Base class for end effector.

    Args:
        cfgs (YACS CfgNode): configurations for the end effector.

    Attributes:
        cfgs (YACS CfgNode): configurations for the end effector.
    """

    def __init__(self, cfgs):
        self.cfgs = cfgs

    def open(self, **kwargs):
        raise NotImplementedError

    def close(self, **kwargs):
        raise NotImplementedError
