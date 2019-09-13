import ast
import importlib
import os


def create_robot(robot_name, pb=True, robot_cfg=None):
    # noinspection SpellCheckingInspection
    """
        create a robot

        :param robot_name: robot name
        :param pb: create robot in Pybullet
        :param robot_cfg: additional configurations for robot
        :type robot_name: String
        :type pb: bool
        :type robot_cfg: dict
        """
    if robot_cfg is None:
        robot_cfg = {}
    root_path = os.path.dirname(os.path.realpath(__file__))
    # noinspection SpellCheckingInspection,SpellCheckingInspection
    cfgs_root_path = os.path.join(root_path, 'cfgs')
    # noinspection SpellCheckingInspection,SpellCheckingInspection
    urdfs_root_path = os.path.join(root_path, 'urdfs')

    robot_pool = []
    for f in os.listdir(cfgs_root_path):
        if f.endswith('_cfg.py'):
            robot_pool.append(f[:-len('_cfg.py')])

    # noinspection SpellCheckingInspection
    root_node = 'airobot.'
    # noinspection SpellCheckingInspection
    cfgs = None
    for tr in robot_pool:
        if tr in robot_name:
            # noinspection SpellCheckingInspection
            mod = importlib.import_module(root_node + 'cfgs.' +
                                          '{:s}_cfg'.format(tr))
            cfg_func = getattr(mod, 'get_cfg')
            # noinspection SpellCheckingInspection
            cfgs = cfg_func()
            break
    if cfgs is None:
        raise ValueError('Invalid robot name provided, only the following'
                         ' robots are available: {}'.format(robot_pool))
    # noinspection SpellCheckingInspection
    urdf = os.path.join(urdfs_root_path,
                        cfgs.URDF)
    # noinspection SpellCheckingInspection
    cfgs.URDF = urdf
    cfgs.freeze()
    robot_path = os.path.join(root_path, 'robot', tr)
    robot_node = root_node + 'robot.' + tr
    # noinspection SpellCheckingInspection,SpellCheckingInspection
    if pb:
        # robot in pybullet should be named with Pybullet suffix
        # noinspection SpellCheckingInspection
        robot_path += '_pybullet'
        # noinspection SpellCheckingInspection
        robot_node += '_pybullet'
    with open(robot_path + '.py') as f:
        node = ast.parse(f.read())
    classes = [n for n in node.body if isinstance(n, ast.ClassDef)]
    mod = importlib.import_module(robot_node)
    # each robot file should have only one class
    robot_class = getattr(mod, classes[0].name)
    robot = robot_class(cfgs, **robot_cfg)
    return robot
