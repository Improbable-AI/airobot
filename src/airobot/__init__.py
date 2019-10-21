import importlib
import os


class Robot:
    def __init__(self,
                 robot_name,
                 pb=True,
                 use_arm=True,
                 use_base=True,
                 use_cam=True,
                 arm_cfg=None,
                 base_cfg=None,
                 cam_cfg=None,
                 eetool_cfg=None
                 ):
        """
        Create a robot instance, which contains:
            arm, base, camera
        End effector tool is inside the arm class

        Args:
            robot_name (str): robot name
            pb (bool): if True, the pybullet simulation
                environment of the robot will be created.
                Otherwise, the ROS environment of the robot
                will be created.
            use_arm (bool): whether to create the robot arm instance
                if the robot has an arm class
            use_base (bool): whether to create the robot base instance
                if the robot has an base class
            use_cam (bool): whether to create the robot camera instance
                if the robot has an camera class
            arm_cfg (dict): arguments to pass in the constructor
                of the arm class
            base_cfg (dict): arguments to pass in the constructor
                of the base class
            cam_cfg (dict): arguments to pass in the constructor
                of the camera class
            eetool_cfg (dict): arguments to pass in the constructor
                of the end effector tool class
        """
        if arm_cfg is None:
            arm_cfg = {}
        if base_cfg is None:
            base_cfg = {}
        if cam_cfg is None:
            cam_cfg = {}
        if eetool_cfg is None:
            eetool_cfg = {}

        root_path = os.path.dirname(os.path.realpath(__file__))
        cfgs_root_path = os.path.join(root_path, 'cfgs')
        robot_pool = []
        for f in os.listdir(cfgs_root_path):
            if f.endswith('_cfg.py'):
                robot_pool.append(f[:-len('_cfg.py')])

        root_node = 'airobot.'
        cfgs = None
        for tr in robot_pool:
            if tr in robot_name:
                mod = importlib.import_module(root_node + 'cfgs.' +
                                              '{:s}_cfg'.format(tr))
                cfg_func = getattr(mod, 'get_cfg')
                cfgs = cfg_func()
                break
        if cfgs is None:
            raise ValueError('Invalid robot name provided, only the following'
                             ' robots are available: {}'.format(robot_pool))

        if pb:
            urdfs_root_path = os.path.join(root_path, 'urdfs')
            urdf = os.path.join(urdfs_root_path,
                                cfgs.PYBULLET_URDF)
            cfgs.PYBULLET_URDF = urdf
        else:
            import rospy
            try:
                rospy.init_node('airobot', anonymous=True)
            except rospy.exceptions.ROSException:
                rospy.logwarn('ROS node [airobot] has already been initialized')

        class_suffix = 'Pybullet' if pb else 'Real'
        if cfgs.HAS_ARM and use_arm:
            mod_path = root_node + 'arm'
            mod = importlib.import_module(mod_path)
            arm_class = getattr(mod, cfgs.ARM.CLASS + class_suffix)
            if cfgs.HAS_EETOOL:
                cfgs.EETOOL.CLASS = cfgs.EETOOL.CLASS + class_suffix
            arm_cfg['eetool_cfg'] = eetool_cfg
            self.arm = arm_class(cfgs, **arm_cfg)
        if cfgs.HAS_BASE and use_base:
            mod_path = root_node + 'base'
            mod = importlib.import_module(mod_path)
            base_class = getattr(mod, cfgs.BASE.CLASS + class_suffix)
            self.base = base_class(cfgs, **base_cfg)
        if cfgs.HAS_CAMERA and use_cam:
            mod_path = root_node + 'sensor.camera'
            mod = importlib.import_module(mod_path)
            camera_class = getattr(mod, cfgs.CAM.CLASS + class_suffix)
            if pb:
                cam_cfg['p'] = self.arm.p
            self.cam = camera_class(cfgs, **cam_cfg)
        cfgs.freeze()
