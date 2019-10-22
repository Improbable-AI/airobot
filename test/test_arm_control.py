import time

import numpy as np
import pytest

from airobot import Robot


@pytest.fixture(scope="module")
def robot_name(request):
    return request.config.getoption('robot_name')


@pytest.fixture(scope="module")
def create_robot(robot_name):
    return Robot(robot_name, pb=False, use_cam=False)


pos_name_pairs = [([0.5, -2, -1.1, -0.95, 1.7, -0.1], None),
                  (-0.5, 'shoulder_pan_joint'),
                  (0, 'shoulder_lift_joint'),
                  (0.4, 'elbow_joint'),
                  (0, 'wrist_1_joint'),
                  (0.5, 'wrist_2_joint'),
                  (0.9, 'wrist_3_joint')]


@pytest.mark.parametrize("position, joint_name", pos_name_pairs)
def test_set_jpos(create_robot, position, joint_name):
    bot = create_robot
    bot.arm.go_home()
    time.sleep(1)
    bot.arm.set_jpos(position, joint_name)
    time.sleep(1)
    jnt_pos = bot.arm.get_jpos(joint_name)
    jnt_pos = np.array(jnt_pos)
    position = np.array(position)
    ag_error = np.fabs(jnt_pos.flatten() - np.array(position).flatten())
    assert np.max(ag_error) < 0.02
