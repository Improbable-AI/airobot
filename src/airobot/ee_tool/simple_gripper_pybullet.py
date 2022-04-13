from airobot.ee_tool.ee import EndEffectorTool


class SimpleGripperPybullet(EndEffectorTool):
    """
    A base class for gripper in pybullet.

    Args:
        cfgs (YACS CfgNode): configurations for the gripper.
        pb_client (BulletClient): pybullet client.

    Attributes:
        cfgs (YACS CfgNode): configurations for the gripper.
        gripper_close_angle (float): position value corresponding to the
            fully closed position of the gripper.
        gripper_open_angle (float): position value corresponding to the
            fully open position of the gripper.
        jnt_names (list): names of the gripper joints.
        gripper_jnt_ids (list): pybullet joint ids of the gripper joints.
        robot_id (int): robot id in Pybullet.
        jnt_to_id (dict): mapping from the joint name to joint id.
    """

    def __init__(self, cfgs, pb_client):
        self._pb = pb_client
        super(SimpleGripperPybullet, self).__init__(cfgs=cfgs)
        self.jnt_names = self.cfgs.EETOOL.JOINT_NAMES
        self._max_torque = self.cfgs.EETOOL.MAX_TORQUE
        self.gripper_close_angle = self.cfgs.EETOOL.CLOSE_ANGLE
        self.gripper_open_angle = self.cfgs.EETOOL.OPEN_ANGLE

        self.deactivate()

    def feed_robot_info(self, robot_id, jnt_to_id):
        """
        Setup the gripper, pass the robot info from the arm to the gripper.

        Args:
            robot_id (int): robot id in Pybullet.
            jnt_to_id (dict): mapping from the joint name to joint id.

        """

        self.robot_id = robot_id
        self.jnt_to_id = jnt_to_id
        self.gripper_jnt_ids = [
            self.jnt_to_id[jnt] for jnt in self.jnt_names
        ]

    def open(self, wait=True, ignore_physics=False):
        """
        Open the gripper.

        Returns:
            bool: return if the action is sucessful or not.
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        success = self.set_jpos(self.gripper_open_angle,
                               wait=wait,
                               ignore_physics=ignore_physics)
        return success

    def close(self, wait=True, ignore_physics=False):
        """
        Close the gripper.

        Returns:
            bool: return if the action is sucessful or not.
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        success = self.set_jpos(self.gripper_close_angle,
                               wait=wait,
                               ignore_physics=ignore_physics)
        return success

    def set_jpos(self, pos, wait=True, ignore_physics=False):
        """
        Set the gripper position.

        Args:
            pos (float): joint position.
            wait (bool): wait until the joint position is set
                to the target position.

        Returns:
            bool: A boolean variable representing if the action is
            successful at the moment when the function exits.
        """
        raise NotImplementedError

    def get_jpos(self, joint_name=None):
        """
        Return the joint position(s) of the arm.

        Args:
            joint_name (str, optional): If it's None,
                it will return joint positions
                of all the actuated joints. Otherwise, it will
                return the joint position of the specified joint.

        Returns:
            One of the following

            - float: joint position given joint_name.
            - list: joint positions if joint_name is None
              (shape: :math:`[DOF]`).
        """
        if joint_name is None:
            states = self._pb.getJointStates(self.robot_id,
                                             self.gripper_jnt_ids)
            pos = [state[0] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            pos = self._pb.getJointState(self.robot_id,
                                         jnt_id)[0]
        return pos

    def get_jvel(self, joint_name=None):
        """
        Return the joint velocity(ies) of the arm.

        Args:
            joint_name (str, optional): If it's None, it will return
                joint velocities of all the actuated joints. Otherwise,
                it will return the joint velocity of the specified joint.

        Returns:
            One of the following

            - float: joint velocity given joint_name.
            - list: joint velocities if joint_name is None
              (shape: :math:`[DOF]`).
        """
        if joint_name is None:
            states = self._pb.getJointStates(self.robot_id,
                                             self.gripper_jnt_ids)
            vel = [state[1] for state in states]
        else:
            jnt_id = self.jnt_to_id[joint_name]
            vel = self._pb.getJointState(self.robot_id,
                                         jnt_id)[1]
        return vel

    def disable_gripper_self_collision(self):
        """
        Disable the gripper collision checking in Pybullet.
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        for i in range(len(self.jnt_names)):
            for j in range(i + 1, len(self.jnt_names)):
                jnt_idx1 = self.jnt_to_id[self.jnt_names[i]]
                jnt_idx2 = self.jnt_to_id[self.jnt_names[j]]
                self._pb.setCollisionFilterPair(self.robot_id,
                                                self.robot_id,
                                                jnt_idx1,
                                                jnt_idx2,
                                                enableCollision=0)

    def deactivate(self):
        """
        Deactivate the gripper.
        """
        self._is_activated = False

    def activate(self):
        """
        Activate the gripper.
        """
        self._is_activated = True

    def _zero_vel_mode(self):
        self._pb.setJointMotorControlArray(self.robot_id,
                                           self.gripper_jnt_ids,
                                           self._pb.VELOCITY_CONTROL,
                                           targetVelocities=[0] * len(self.gripper_jnt_ids),
                                           forces=[10] * len(self.gripper_jnt_ids))

    def _hard_reset(self, pos):
        for i in range(min(len(self.gripper_jnt_ids), len(pos))):
            self._pb.resetJointState(self.robot_id,
                                     self.gripper_jnt_ids[i],
                                     targetValue=pos[i],
                                     targetVelocity=0)
