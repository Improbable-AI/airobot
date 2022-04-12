from airobot.ee_tool.simple_gripper_pybullet import SimpleGripperPybullet


class Robotiq2F140Pybullet(SimpleGripperPybullet):
    """
    Class for interfacing with a Robotiq 2F140 gripper in pybullet.

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
        super(Robotiq2F140Pybullet, self).__init__(cfgs=cfgs,
                                                   pb_client=pb_client)

    def _setup_gripper(self):
        """
        Setup the gripper, pass the robot info from the arm to the gripper.

        Args:
            robot_id (int): robot id in Pybullet.
            jnt_to_id (dict): mapping from the joint name to joint id.

        """
        self._pb.changeDynamics(self.robot_id,
                                self.jnt_to_id['left_inner_finger_pad_joint'],
                                lateralFriction=2.0,
                                spinningFriction=1.0,
                                rollingFriction=1.0)
        self._pb.changeDynamics(self.robot_id,
                                self.jnt_to_id['right_inner_finger_pad_joint'],
                                lateralFriction=2.0,
                                spinningFriction=1.0,
                                rollingFriction=1.0)

    def feed_robot_info(self, robot_id, jnt_to_id):
        """
        Setup the gripper, pass the robot info from the arm to the gripper.

        Args:
            robot_id (int): robot id in Pybullet.
            jnt_to_id (dict): mapping from the joint name to joint id.

        """
        c_mimic = self._pb.createConstraint(
            self.robot_id,
            self.jnt_to_id['finger_joint'],
            self.robot_id,
            self.jnt_to_id['right_outer_knuckle_joint'],
            jointType=self._pb.JOINT_GEAR,
            jointAxis=[1, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=[0, 0, 0])
        self._pb.changeConstraint(c_mimic, gearRatio=1.0, erp=0.2, maxForce=10)

        parent_pos = [0.0, -0.01, -0.004]
        child_pos = [0, 0.049, 0.0]
        c1 = self._pb.createConstraint(
            self.robot_id,
            self.jnt_to_id['right_inner_finger_joint'],
            self.robot_id,
            self.jnt_to_id['right_inner_knuckle_joint'],
            jointType=self._pb.JOINT_POINT2POINT,
            jointAxis=[1, 0, 0], 
            parentFramePosition=parent_pos,
            childFramePosition=child_pos)
        self._pb.changeConstraint(c1, erp=0.8, maxForce=9999)

        c2 = self._pb.createConstraint(
            self.robot_id,
            self.jnt_to_id['left_inner_finger_joint'],
            self.robot_id,
            self.jnt_to_id['left_inner_knuckle_joint'],
            jointType=self._pb.JOINT_POINT2POINT,
            jointAxis=[1, 0, 0], 
            parentFramePosition=parent_pos,
            childFramePosition=child_pos)
        self._pb.changeConstraint(c2, erp=0.8, maxForce=9999)

        self._pb.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['right_inner_finger_joint'], controlMode=self._pb.VELOCITY_CONTROL, force=0)
        self._pb.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['left_inner_finger_joint'],  controlMode=self._pb.VELOCITY_CONTROL, force=0)
        self._pb.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['right_inner_knuckle_joint'],  controlMode=self._pb.VELOCITY_CONTROL, force=0)
        self._pb.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['left_inner_knuckle_joint'],  controlMode=self._pb.VELOCITY_CONTROL, force=0)
        # self._pb.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['gripper_right_coupler_joint'],  controlMode=self._pb.VELOCITY_CONTROL, force=0)
        # self._pb.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['gripper_left_coupler_joint'],  controlMode=self._pb.VELOCITY_CONTROL, force=0)
        
        self._pb.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['right_outer_knuckle_joint'],  controlMode=self._pb.VELOCITY_CONTROL, force=0)

        self._setup_gripper()

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
        joint_name = self.jnt_names[0]
        tgt_pos = arutil.clamp(
            pos,
            min(self.gripper_open_angle, self.gripper_close_angle),
            max(self.gripper_open_angle, self.gripper_close_angle))

        jnt_id = self.jnt_to_id[joint_name]

        # self._pb.setJointMotorControl2(targetPosition=-1.0*self.MAX_CLOSING, bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['right_inner_knuckle_joint'], controlMode=self._pb.POSITION_CONTROL, force=1)
        # self._pb.setJointMotorControl2(targetPosition=1.0*self.MAX_CLOSING, bodyIndex=self.robot_id, jointIndex=self.jnt_to_id['left_inner_knuckle_joint'], controlMode=self._pb.POSITION_CONTROL, force=1)

        self._pb.setJointMotorControl2(self.robot_id,
            jnt_id,
            self._pb.POSITION_CONTROL,
            targetPosition=tgt_pos,
            force=self._max_torque)

        return True

    def get_jpos(self):
        """
        Return the joint position(s) of the gripper.

        Returns:
            float: joint position.
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        jnt_id = self.jnt_to_id[self.jnt_names[0]]
        pos = self._pb.getJointState(self.robot_id, jnt_id)[0]
        return pos

    def get_jvel(self):
        """
        Return the joint velocity of the gripper.

        Returns:
            float: joint velocity.
        """
        if not self._is_activated:
            raise RuntimeError('Call activate function first!')
        jnt_id = self.jnt_to_id[self.jnt_names[0]]
        vel = self._pb.getJointState(self.robot_id, jnt_id)[1]
        return vel

