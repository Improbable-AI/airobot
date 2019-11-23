import os
import time

import numpy as np

from airobot import Robot
from airobot.utils.common import euler2quat
from airobot.utils.pb_util import TextureModder
from airobot.utils.pb_util import load_geom, load_urdf


def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions
    """
    dir_path = os.path.dirname(os.path.realpath(__file__))
    texture_path = os.path.join(dir_path, 'textures')
    robot = Robot('ur5e', arm_cfg={'render': True})
    robot.arm.go_home()

    modder = TextureModder()

    ori = euler2quat([0, 0, np.pi / 2])
    table_id = load_urdf('table/table.urdf', [1, 0, 0.4], ori, scaling=0.9)
    sphere_id = load_geom('sphere', size=0.05, mass=1,
                          base_pos=[1, 0, 1.0], rgba=[0, 1, 0, 1])
    box_id = load_geom('box', size=0.05, mass=1,
                       base_pos=[1, 0.12, 1.0], rgba=[1, 0, 0, 1])
    duck_id = load_geom('mesh', mass=1, visualfile='duck.obj',
                        mesh_scale=0.1,
                        base_pos=[0.9, -0.4, 1.0],
                        rgba=[0.5, 0.2, 1, 1])
    modder.set_texture(table_id, -1, os.path.join(texture_path, '1.jpg'))
    modder.set_texture(sphere_id, -1, os.path.join(texture_path, '2.jpg'))
    modder.set_texture(box_id, -1, os.path.join(texture_path, '3.jpg'))
    modder.set_texture(duck_id, -1, os.path.join(texture_path, '4.jpg'))
    modder.set_texture_path(texture_path)

    while True:
        start = time.time()
        # modder.randomize('rgb')
        # modder.randomize('gradient')
        # modder.randomize('noise')
        # modder.randomize('texture', exclude={robot.arm.robot_id: []})
        # modder.randomize('texture',
        #                  exclude={robot.arm.robot_id: [3, 4, 5, 6]})
        modder.randomize('all')
        print('Time cost (s): ', time.time() - start)
        time.sleep(1)


if __name__ == '__main__':
    main()
