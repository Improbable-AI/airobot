import time
import numpy as np
from airobot import Robot
from airobot.utils.pb_util import load_geom, load_mjcf, load_sdf, load_urdf, set_step_sim
from airobot.utils.common import euler2quat

def main():
    """
    This function demonstrates how to move the robot arm
    to the desired joint positions
    """
    robot = Robot('ur5e', arm_cfg={'render': True})
    robot.arm.go_home()

    ori = euler2quat([0, 0, np.pi/2])
    load_urdf('table/table.urdf', [1, 0, 0.4], ori, scaling=0.9)
    # load_mjcf('mjcf/ant.xml')
    # load_sdf('kuka_iiwa/kuka_with_gripper.sdf')
    load_geom('sphere', size=0.05, mass=1,
              base_pos=[1, 0, 1.0], rgba=[0, 1, 0, 1])
    load_geom('box', size=0.05, mass=1,
              base_pos=[1, 0.12, 1.0], rgba=[1, 0, 0, 1])
    load_geom('box', size=[0.06, 0.02, 0.03], mass=1,
              base_pos=[1.3, 0.12, 1.0], rgba=[0, 0, 1, 1])
    load_geom('cylinder', size=[0.06, 0.08], mass=1,
              base_pos=[0.8, -0.12, 1.0], rgba=[0, 1, 1, 1])
    load_geom('mesh', mass=1, filename='duck.obj',
              mesh_scale=0.1,
              base_pos=[0.9, -0.4, 1.0],
              rgba=[0.5, 0.2, 1, 1])

    time.sleep(10)


if __name__ == '__main__':
    main()
