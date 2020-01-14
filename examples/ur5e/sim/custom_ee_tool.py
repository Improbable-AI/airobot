import time

import os
from airobot import Robot
from airobot.utils.common import euler2quat
from airobot.utils.pb_util import load_urdf
from airobot.utils.pb_util import load_geom
import numpy as np


def main():
    """
    This function demonstrates how to load a custom
    end effector to the UR5e robot in pybullet
    """
    robot = Robot('ur5e',
                  pb=True,
                  use_eetool=False,
                  arm_cfg={'render': True, 'self_collision': False})
    robot.arm.go_home()

    ori = euler2quat([0, 0, np.pi / 2])
    table_id = load_urdf('table/table.urdf',
                         [1, 0, 0.4],
                         ori,
                         scaling=0.9)

    box_id = load_geom('box',
                       size=0.05,
                       mass=1,
                       base_pos=[0.6, 0.12, 1.0],
                       rgba=[1, 0, 0, 1])
    box_id2 = load_geom('box',
                        size=[0.06, 0.02, 0.03],
                        mass=1,
                        base_pos=[0.6, -0.12, 1.0],
                        rgba=[0, 0, 1, 1])

    # custom EE tool can either be added by making a new separate URDF
    
    root_path = os.path.dirname(os.path.realpath(__file__))
    stick_id = load_urdf(
        os.path.join(root_path,
                     "../../../src/airobot/urdfs/custom_ee_tools/stick_ee_tool.urdf"),
        [0.3, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    )

    # or using the load_geom capability in the pb_util file
    
    # stick_id = load_geom('cylinder', size=[0.015, 0.1524], mass=0.01,
    #                      base_pos=[0.3, 0.0, 0.0], rgba=[0, 1, 1, 1])

    cid = robot.arm.p.createConstraint(
        robot.arm.robot_id,
        robot.arm.ee_link_id,
        stick_id,
        -1,
        robot.arm.p.JOINT_FIXED,
        jointAxis=[1, 1, 1],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, -0.077],
        childFrameOrientation=euler2quat([0, 0, 0])
    )

    time.sleep(5)


if __name__ == '__main__':
    main()
