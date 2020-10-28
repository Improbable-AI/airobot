"""
An ABB Yumi robot arm
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import numbers
import time

import airobot as ar
import airobot.utils.common as arutil
import numpy as np
import rospy
from airobot.arm.single_arm_ros import SingleArmROS
from airobot.utils.arm_util import wait_to_reach_ee_goal
from airobot.utils.arm_util import wait_to_reach_jnt_goal
from airobot.utils.common import print_red
from airobot.utils.moveit_util import moveit_cartesian_path
from airobot.utils.ros_util import kdl_frame_to_numpy, get_tf_transform
# import airobot.utils.srv.yumi as srv
import abb_robotnode.srv as srv
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class YumiArmReal(SingleArmROS):
    """
    A Class for interfacing with a single arm of a real Yumi robot.

    Args:
        cfgs (YACS CfgNode): configurations for the arm.
        moveit_planner (str): motion planning algorithm.
        eetool_cfg (dict): arguments to pass in the constructor
            of the end effector tool class.
        wrist_cam (bool): whether the robot has a wrist camera
            mounted. If so, a box will be placed around the camera
            so that moveit is aware of the wrist camera when it's
            doing motion planning.

    Attributes:
    """

    def __init__(self, cfgs,
                 moveit_planner='RRTstarkConfigDefault',
                 eetool_cfg=None,
                 wrist_cam=True):
        super(YumiArmReal, self).__init__(cfgs=cfgs,
                                          moveit_planner=moveit_planner,
                                          eetool_cfg=eetool_cfg)
        self._has_wrist_cam = wrist_cam
        self._egm_active = False
        self._init_yumi_consts()

        if not self._gazebo_sim:
            # self.robot_ip = rospy.get_param('robot_ip')
            self._setup_srv()
            self._setup_pub_sub()
            self._set_tool_offset()

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot (units in rad).

        Args:
            position (float or list or flattened np.ndarray):
                desired joint position(s) in RADIANS
                (shape: :math:`[DOF,]` if list, otherwise a single value).
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move. Defaults to None.
            wait (bool): whether position command should be blocking or non
                blocking. Defaults to True.

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        position = copy.deepcopy(position)
        success = False

        if joint_name is None:
            if len(position) != self.arm_dof:
                raise ValueError('position should contain %d elements if '
                                 'joint_name is not provided' % self.arm_dof)
            tgt_pos = position
        else:
            if not isinstance(position, numbers.Number):
                raise TypeError('position should be individual float value'
                                ' if joint_name is provided')
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = self.get_jpos()
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                tgt_pos[arm_jnt_idx] = position

        # if self._use_moveit:
            # self.moveit_group.set_joint_value_target(tgt_pos)
            # plan = self.moveit_group.plan()

            # #
            # success = self.moveit_group.go(tgt_pos, wait=wait)
        if self._egm_active:
            joint_msg = self._empty_joint_state_msg()
            joint_msg.position = self._flip_joints_set(np.rad2deg(tgt_pos))
            self._egm_target_joints_pub.publish(joint_msg)
        else:
            rospy.wait_for_service(
                self.cfgs.ARM.SET_JOINTS_SRV,
                timeout=self._srv_timeout)
            self._set_joints_srv(self._flip_joints_set(np.rad2deg(tgt_pos)))

        if wait:
            success = wait_to_reach_jnt_goal(
                position,
                get_func=self.get_jpos,
                joint_name=joint_name,
                timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                max_error=self.cfgs.ARM.MAX_JOINT_ERROR
            )
        return success

    def set_jpos_buffer(self, pos_buffer, joint_name=None,
                        wait=True, sync=True, execute=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot (units in rad).

        Args:
            position (float or list or flattened np.ndarray):
                desired joint position(s)
                (shape: :math:`[DOF,]` if list, otherwise a single value).
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move. Defaults to None.
            wait (bool): whether position command should be blocking or non
                blocking. Defaults to True.

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        if len(pos_buffer) == 1:
            pos_buffer.append(pos_buffer[0])

        pos_buffer = copy.deepcopy(pos_buffer)
        success = False

        if joint_name is None:
            if len(pos_buffer[0]) != self.arm_dof:
                raise ValueError('position should contain %d elements if '
                                 'joint_name is not provided' % self.arm_dof)
            tgt_pos_buffer = pos_buffer
        else:
            if not isinstance(pos_buffer, numbers.Number):
                raise TypeError('position should be individual float value'
                                ' if joint_name is provided')
            if joint_name not in self.arm_jnt_names:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = self.get_jpos()
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                tgt_pos[arm_jnt_idx] = pos_buffer

        # rospy.wait_for_service(self._clear_buffer_srv,
        #                        timeout=self._srv_timeout)
        # rospy.wait_for_service(self._add_to_buffer_srv,
        #                        timeout=self._srv_timeout)
        rospy.wait_for_service(self.cfgs.ARM.CLEAR_BUFFER_SRV,
                               timeout=self._srv_timeout)
        rospy.wait_for_service(self.cfgs.ARM.ADD_BUFF_SRV,
                               timeout=self._srv_timeout)

        self._clear_buffer_srv()
        for tgt_pos in tgt_pos_buffer:
            self._add_to_buffer_srv(self._flip_joints_set(np.rad2deg(tgt_pos)))

        if execute:
            if sync:
                rospy.wait_for_service(self.cfgs.ARM.EXEC_BUFF_SYNC_SRV,
                                    timeout=self._srv_timeout)
                exec_buffer = self._execute_buffer_sync_srv
            else:
                rospy.wait_for_service(self.cfgs.ARM.EXEC_BUFF_SRV,
                                    timeout=self._srv_timeout)
                exec_buffer = self._execute_buffer_srv
            exec_buffer()

        success = False
        if wait:
            success = wait_to_reach_jnt_goal(
                tgt_pos_buffer[-1],
                get_func=self.get_jpos,
                joint_name=joint_name,
                timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                max_error=self.cfgs.ARM.MAX_JOINT_ERROR
            )
        return success

    def set_ee_pose(self, pos=None, ori=None, wait=True,
                    ik_first=False, *args, **kwargs):
        """
        Set cartesian space pose of end effector.

        Args:
            pos (list or np.ndarray): Desired x, y, z positions in the robot's
                base frame to move to (shape: :math:`[3,]`).
            ori (list or np.ndarray, optional): It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation.
            wait (bool): wait until the motion completes.
            ik_first (bool, optional): Whether to use the solution computed
                by IK, or to use UR built in movel function which moves
                linearly in tool space (movel may sometimes fail due to
                sinularities). This parameter takes effect only when
                self._use_urscript is True. Defaults to False.

        Returns:
            bool: Returns True is robot successfully moves to goal pose.
        """
        if ori is None and pos is None:
            return True
        success = False
        if ori is None:
            pose = self.get_ee_pose()  # last index is euler angles
            quat = pose[1]
        else:
            quat = arutil.to_quat(ori)
        if pos is None:
            pose = self.get_ee_pose()
            pos = pose[0]

        if self._use_moveit:
            pose = self.moveit_group.get_current_pose()
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            self.moveit_group.set_pose_target(pose)
            success = self.moveit_group.go(wait=wait)
        else:
            if ik_first:
                jnt_pos = self.compute_ik(pos, quat)
                success = self.set_jpos(jnt_pos, wait=wait)
            else:
                pos = np.asarray(pos)*1000.0

                rospy.wait_for_service(self.cfgs.ARM.SET_CARTESIAN_SRV,
                                       timeout=self._srv_timeout)
                self._set_cartesian_srv(
                    pos[0],
                    pos[1],
                    pos[2],
                    quat[3],
                    quat[0],
                    quat[1],
                    quat[2])
                if wait:
                    args_dict = {
                        'get_func': self.get_ee_pose,
                        'get_func_derv': None,
                        'timeout': self.cfgs.ARM.TIMEOUT_LIMIT,
                        'pos_tol': self.cfgs.ARM.MAX_EE_POS_ERROR,
                        'ori_tol': self.cfgs.ARM.MAX_EE_ORI_ERROR
                    }
                    success = wait_to_reach_ee_goal(pos, quat,
                                                    **args_dict)
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, wait=True,
                    *args, **kwargs):
        """
        Move end effector in straight line while maintaining orientation.

        Args:
            delta_xyz (list or np.ndarray): Goal change in x, y, z position of
                end effector.
            eef_step (float, optional): Discretization step in cartesian space
                for computing waypoints along the path. Defaults to 0.005 (m).
            wait (bool, optional): True if robot should not do anything else
                until this goal is reached, or the robot times out.
                Defaults to True.

        Returns:
            bool: True if robot successfully reached the goal pose.
        """
        ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()

        # if self._use_urscript:
        ee_pos[0] += delta_xyz[0]
        ee_pos[1] += delta_xyz[1]
        ee_pos[2] += delta_xyz[2]
        success = self.set_ee_pose(ee_pos, ee_euler, wait=wait,
                                   ik_first=False)
        # else:
        #     plan = moveit_cartesian_path(ee_pos,
        #                                  ee_quat,
        #                                  delta_xyz,
        #                                  self.moveit_group,
        #                                  eef_step)
        #     success = self.moveit_group.execute(plan, wait=wait)
        return success

    def get_jpos(self, joint_name=None):
        """
        Gets the current joint position of the robot. Gets the value
        from the internally updated dictionary that subscribes to the ROS
        topic /joint_states.

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
        if self._egm_active:
            # comes in as radians!
            self._j_state_lock.acquire()
            if joint_name is not None:
                if joint_name not in self.arm_jnt_names:
                    raise TypeError('Joint name [%s] '
                                    'not recognized!' % joint_name)
                jpos = self._j_pos[joint_name]
            else:
                jpos = []
                for joint in self.arm_jnt_names:
                    jpos.append(self._j_pos[joint])
            self._j_state_lock.release()
        else:
            rospy.wait_for_service(self.cfgs.ARM.GET_JOINTS_SRV,
                                   timeout=self._srv_timeout)
            joint_positions = self._get_joints_srv()
            # comes in as tuple. we need to 1. reorder, 2. convert deg2rad, 3. convert to list
            joint_positions = self._flip_joints_get(list(joint_positions.joints))
            joint_positions = np.deg2rad(joint_positions).tolist()
            if joint_name is not None:
                if joint_name not in self.arm_jnt_names:
                    raise TypeError('Joint name [%s] '
                                    'not recognized!' % joint_name)
                jnt_ind = self.arm_jnt_names.index(joint_name)
                jpos = joint_positions[jnt_ind]
            else:
                jpos = joint_positions
        return jpos

    def get_ee_pose(self):
        """
        Get current cartesian pose of the EE, in the robot's base frame,
        using ROS subscriber to the tf tree topic.

        Returns:
            4-element tuple containing

            - np.ndarray: x, y, z position of the EE (shape: :math:`[3]`).
            - np.ndarray: quaternion representation ([x, y, z, w]) of the EE
              orientation (shape: :math:`[4]`).
            - np.ndarray: rotation matrix representation of the EE orientation
              (shape: :math:`[3, 3]`).
            - np.ndarray: euler angle representation of the EE orientation
              (roll, pitch, yaw with static reference frame)
              (shape: :math:`[3]`).
        """
        if self._egm_active:
            pos, quat = get_tf_transform(self.tf_listener,
                                         self.cfgs.ARM.ROBOT_BASE_FRAME,
                                         self.cfgs.ARM.ROBOT_EE_FRAME)
        else:
            rospy.wait_for_service(self.cfgs.ARM.GET_CARTESIAN_SRV,
                                   timeout=self._srv_timeout)
            ee_pose = self._get_cartesian_srv()
            # scale from mm to m
            pos = np.asarray([ee_pose.x, ee_pose.y, ee_pose.z]) / 1000.0
            pos = pos.tolist()

            quat = [ee_pose.qx, ee_pose.qy, ee_pose.qz, ee_pose.q0]
        rot_mat = arutil.quat2rot(quat)
        euler_ori = arutil.quat2euler(quat)
        return np.array(pos), np.array(quat), rot_mat, euler_ori

    def egm_is_active(self):
        return self._egm_active

    def start_egm(self):
        """
        Turn on EGM mode
        """
        rospy.wait_for_service(self.cfgs.ARM.EGM_MODE_SRV,
                               timeout=self._srv_timeout)
        self._set_egm_mode_srv(5)
        self._egm_active = True

    def stop_egm(self):
        """
        Turn off EGM mode
        """
        rospy.wait_for_service(self.cfgs.ARM.EGM_MODE_SRV,
                               timeout=self._srv_timeout)
        self._set_egm_mode_srv(0)
        self._egm_active = False

    def set_speed(self, speed_tcp, speed_ori, speed_joints=180):
        rospy.wait_for_service(self.cfgs.ARM.SET_SPEED_SRV,
                               timeout=self._srv_timeout)
        self._set_speed_srv(speed_tcp, speed_ori, speed_joints)

    def _flip_joints_set(self, jpos):
        """
        Assuming joints come in order that is specified in the cfg file,
        need to reorder them before sending them to the real robot. 
        Joint order on the robot is [1, 2, 3, 4, 5, 6, 7]

        Args:
            jpos (list): Joint positions in the order specified in the cfg file

        Returns: 
            list: Joint positions with same values but reordered to match real yumi
        """
        # jpos_flipped = copy.deepcopy(jpos)
        jpos_flipped = [
            jpos[0],
            jpos[1],
            jpos[3],
            jpos[4],
            jpos[5],
            jpos[6],
            jpos[2]            ,
        ]
        return jpos_flipped

    def _flip_joints_get(self, jpos):
        """
        Assuming joints come in order that is specified in the cfg file,
        need to reorder them before sending them to the real robot. 
        Joint order on the robot is [1, 2, 3, 4, 5, 6, 7]

        Args:
            jpos (list): Joint positions in the order specified in the cfg file

        Returns: 
            list: Joint positions with same values but reordered to match real yumi
        """
        # jpos_flipped = copy.deepcopy(jpos)
        # jpos_flipped[0] = jpos[0]
        # jpos_flipped[1] = jpos[1]
        # jpos_flipped[2] = jpos[6]
        # jpos_flipped[3] = jpos[2]
        # jpos_flipped[4] = jpos[3]
        # jpos_flipped[5] = jpos[4]
        # jpos_flipped[6] = jpos[5]
        jpos_flipped = [
            jpos[0],
            jpos[1],
            jpos[6],
            jpos[2],
            jpos[3],
            jpos[4],
            jpos[5]            ,
        ]        
        return jpos_flipped        

    def _empty_joint_state_msg(self):
        """
        Return a JointState message that is ready to be filled with values,
        but which already has the name field filled in

        Returns:
            JointState: JointState message with name field filled and other
                fields empty
        """
        joint_msg = JointState()
        # need to flip joints 3 and 7!
        joint_msg.name = [''] * self.arm_dof
        joint_msg.name[0] = self.arm_jnt_names[0]
        joint_msg.name[1] = self.arm_jnt_names[1]
        joint_msg.name[2] = self.arm_jnt_names[3]
        joint_msg.name[3] = self.arm_jnt_names[4]
        joint_msg.name[4] = self.arm_jnt_names[5]
        joint_msg.name[5] = self.arm_jnt_names[6]
        joint_msg.name[6] = self.arm_jnt_names[2]   
        # joint_msg.name = self.arm_jnt_names                     
        return joint_msg

    def _output_pendant_msg(self, msg):
        """
        Method to display a text message on the UR5e teach pendant.

        Args:
            msg (str): message to display.

        """
        raise NotImplementedError

    def _init_yumi_consts(self):
        """
        Initialize real yumi specific contants
        """
        self._srv_timeout = self.cfgs.ARM.SERVICE_TIMEOUT_DEFAULT
        self._use_moveit = False  # TODO

    def _get_tip_transform(self):
        """
        Internal method to get the transform between the robot's
        wrist and the tip of the gripper.

        Returns:
            2-element tuple containing

            - list: Translation component of the gripper tip transform
              (shape :math:`[3,]`).
            - list: Euler angle orientation component of the gripper
              tip transform. (shape :math:`[3,]`).
        """
        ee_frame = self.cfgs.ARM.ROBOT_EE_FRAME
        gripper_tip_id = self.arm_link_names.index(ee_frame)
        gripper_tip_link = self._urdf_chain.getSegment(gripper_tip_id)
        gripper_tip_tf = kdl_frame_to_numpy(gripper_tip_link.getFrameToTip())
        gripper_tip_pos = gripper_tip_tf[:3, 3].flatten()
        gripper_tip_rot_mat = gripper_tip_tf[:3, :3]
        gripper_tip_euler = arutil.rot2euler(gripper_tip_rot_mat)
        return list(gripper_tip_pos), list(gripper_tip_euler)

    def _set_tool_offset(self):
        """
        Internal method to send a URScript command to the robot so that
        it updates it tool center point variable to match the URDF.
        """
        # raise NotImplementedError
        pass

    def _setup_srv(self):
        """
        Initialize all rospy service proxy interfaces
        """
        self._set_joints_srv = rospy.ServiceProxy(
            self.cfgs.ARM.SET_JOINTS_SRV,
            srv.Service_SetJoints)

        self._get_joints_srv = rospy.ServiceProxy(
            self.cfgs.ARM.GET_JOINTS_SRV,
            srv.Service_GetJoints)

        self._set_cartesian_srv = rospy.ServiceProxy(
            self.cfgs.ARM.SET_CARTESIAN_SRV,
            srv.Service_SetCartesian)

        self._get_cartesian_srv = rospy.ServiceProxy(
            self.cfgs.ARM.GET_CARTESIAN_SRV,
            srv.Service_GetCartesian)

        self._set_speed_srv = rospy.ServiceProxy(
            self.cfgs.ARM.SET_SPEED_SRV,
            srv.Service_SetMaxSpeed)

        self._set_egm_mode_srv = rospy.ServiceProxy(
            self.cfgs.ARM.EGM_MODE_SRV,
            srv.Service_SetEGMMode)

        self._add_to_buffer_srv = rospy.ServiceProxy(
            self.cfgs.ARM.ADD_BUFF_SRV,
            srv.Service_AddToJointBuffer)

        self._clear_buffer_srv = rospy.ServiceProxy(
            self.cfgs.ARM.CLEAR_BUFF_SRV,
            srv.Service_ClearJointBuffer)

        self._execute_buffer_sync_srv = rospy.ServiceProxy(
            self.cfgs.ARM.EXEC_BUFF_SYNC_SRV,
            srv.Service_ExecuteSynchroJointBuffer)

        self._execute_buffer_srv = rospy.ServiceProxy(
            self.cfgs.ARM.EXEC_BUFF_SRV,
            srv.Service_ExecuteJointBuffer)

    def _setup_pub_sub(self):
        """
        Initialize all the publishers and subscribers used internally.
        """
        self._egm_target_joints_pub = rospy.Publisher(
            self.cfgs.ARM.EGM_TARGET_JOINTS_TOPIC,
            JointState,
            queue_size=20
        )

        # wait a second for pub/sub to connect
        time.sleep(1)
