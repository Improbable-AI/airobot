import os
import time

import pybullet as p

from airobot import Robot
from airobot.utils.common import euler2quat


def main():
    """
    This function demonstrates how to load a custom
    end effector to the UR5e robot in pybullet.

    NOTE: This implementation using PyBullet's createConstraint
    function is known to have issues with fixed joints (fixed joints
    are not actually rigidly fixed).
    """
    robot = Robot('ur5e',
                  pb=True,
                  use_eetool=False,
                  arm_cfg={'self_collision': False})
    robot.arm.go_home()

    # custom EE tool can either be added by making a new separate URDF
    # or using the load_geom capbility in pb_util file

    # uncomment below to load an example custom URDF
    root_path = os.path.dirname(os.path.realpath(__file__))
    urdf_path = '../../../src/airobot/urdfs/custom_ee_tools/stick_ee_tool.urdf'
    stick_id = robot.pb_client.load_urdf(
        os.path.join(root_path, urdf_path),
        [0.3, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    )

    # uncomment below to load a basic geometry with load_geom
    # from airobot.utils.pb_util import load_geom
    # stick_id = load_geom('cylinder', size=[0.015, 0.1524], mass=0.01,
    #                      base_pos=[0.3, 0.0, 0.0], rgba=[0, 1, 1, 1])

    robot.pb_client.createConstraint(
        robot.arm.robot_id,
        robot.arm.ee_link_id,
        stick_id,
        -1,
        p.JOINT_FIXED,
        jointAxis=[1, 1, 1],
        parentFramePosition=[0, 0, 0],
        childFramePosition=[0, 0, -0.077],
        childFrameOrientation=euler2quat([0, 0, 0]))

    time.sleep(5)


if __name__ == '__main__':
    main()
