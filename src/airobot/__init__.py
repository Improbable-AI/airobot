import ast
import importlib
import os


def create_robot(robot_name, pb=True, robot_cfg=None):
    """
    Create a robot based on the robot name
    Args:
        robot_name (str): robot name
        pb (bool): if True, the pybullet simulation
            environment of the robot will be created.
            Otherwise, the ROS environment of the robot
            will be created.
        robot_cfg (dict): additioinal robot configurations
            to be passed into the Robot class.

    Returns:
        An robot instance

    """

    if robot_cfg is None:
        robot_cfg = {}

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
    cfgs.freeze()
    robot_path = os.path.join(root_path, 'robot', tr)
    robot_node = root_node + 'robot.' + tr
    if pb:
        # robot in pybullet should be named with _pybullet suffix
        robot_path += '_pybullet'
        robot_node += '_pybullet'
    else:
        # real robot should be named with _real suffix
        robot_path += '_real'
        robot_node += '_real'
    with open(robot_path + '.py') as f:
        node = ast.parse(f.read())
    classes = [n for n in node.body if isinstance(n, ast.ClassDef)]
    mod = importlib.import_module(robot_node)
    # each robot file should have only one class
    robot_class = getattr(mod, classes[0].name)
    robot = robot_class(cfgs, **robot_cfg)
    return robot
