"""
A UR5e robot arm
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import numbers
import time

import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import airobot as ar
import airobot.utils.common as arutil
from airobot.arm.single_arm_ros import SingleArmROS
from airobot.utils.arm_util import wait_to_reach_ee_goal
from airobot.utils.arm_util import wait_to_reach_jnt_goal
from airobot.utils.common import print_red
from airobot.utils.moveit_util import moveit_cartesian_path
from airobot.utils.ros_util import kdl_frame_to_numpy


class UR5eReal(SingleArmROS):
    def __init__(self, cfgs,
                 moveit_planner='RRTstarkConfigDefault',
                 eetool_cfg=None,
                 wrist_cam=True):
        """

        Args:
            cfgs (YACS CfgNode): configurations for the arm
            moveit_planner (str): motion planning algorithm
            eetool_cfg (dict): arguments to pass in the constructor
                of the end effector tool class
            wrist_cam (bool): whether the robot has a wrist camera
                mounted. If so, a box will be placed around the camera
                so that moveit is aware of the wrist camera when it's
                doing motion planning
        """
        super(UR5eReal, self).__init__(cfgs=cfgs,
                                       moveit_planner=moveit_planner,
                                       eetool_cfg=eetool_cfg)
        self.has_wrist_cam = wrist_cam
        self._init_ur_consts()

        if not self.gazebo_sim:
            self.robot_ip = rospy.get_param('robot_ip')
            self.set_comm_mode()
            self._setup_pub_sub()
            self._set_tool_offset()
        else:
            self.set_comm_mode(use_urscript=False)
        self.is_jpos_in_good_range()

    def is_jpos_in_good_range(self):
        """
        Check if the joint angles lie in (-pi, pi]

        Returns:
            bool: whether the joint angles are in (-pi, pi]

        """
        jposs = self.get_jpos()
        for i, jpos in enumerate(jposs):
            if jpos <= -np.pi or jpos > np.pi:
                ar.log_warn('Current joint angles are: %s\n'
                            'Some joint angles are outside of the valid'
                            ' range (-pi, pi]\n Please use the Teaching'
                            ' Pendant to move the correponding joints so'
                            ' that all the joint angles are within (-pi,'
                            ' pi]!' % str(jposs))
                return False
        return True

    def set_comm_mode(self, use_urscript=False):
        """
        Method to set whether to use ros or urscript to control the real robot.
        In gazebo, it's always False.

        Args:
            use_urscript (bool): True we should use urscript
                False if we should use ros and moveit
        """
        if self.gazebo_sim:
            self.use_urscript = False
            arutil.print_yellow('Use urscript is not supported in Gazebo!')
        else:
            self.use_urscript = use_urscript

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot (units in rad)

        Args:
            position (float or list or flattened np.ndarray):
                desired joint position(s)
                (shape: :math:`[6,]` if list, otherwise a single value)
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move. Defaults to None
            wait (bool): whether position command should be blocking or non
                blocking. Defaults to True

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
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_pos = self.get_jpos()
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                tgt_pos[arm_jnt_idx] = position
        if self.use_urscript:
            prog = 'movej([%f, %f, %f, %f, %f, %f],' \
                   ' a=%f, v=%f)' % (tgt_pos[0],
                                     tgt_pos[1],
                                     tgt_pos[2],
                                     tgt_pos[3],
                                     tgt_pos[4],
                                     tgt_pos[5],
                                     self._motion_acc,
                                     self._motion_vel)
            self._send_urscript(prog)

            if wait:
                success = wait_to_reach_jnt_goal(
                    position,
                    get_func=self.get_jpos,
                    joint_name=joint_name,
                    get_func_derv=self.get_jvel,
                    timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                    max_error=self.cfgs.ARM.MAX_JOINT_ERROR
                )
        else:
            self.moveit_group.set_joint_value_target(tgt_pos)
            success = self.moveit_group.go(tgt_pos, wait=wait)

        return success

    def set_jvel(self, velocity, joint_name=None, wait=False,
                 *args, **kwargs):
        """
        Set joint velocity command to the robot (units in rad/s)

        Args:
            velocity (float or list or flattened np.ndarray): list of target
                joint velocity value(s)
                (shape: :math:`[6,]` if list, otherwise a single value)
            joint_name (str, optional): If not provided, velocity should be
                list and all joints will be turned on at specified velocity.
                Defaults to None.
            wait (bool, optional): If True, block until robot reaches
                desired joint velocity value(s). Defaults to False.

        Returns:
            bool: True if command was completed successfully, returns
            False if wait flag is set to False.
        """
        velocity = copy.deepcopy(velocity)
        success = False

        if self.gazebo_sim:
            raise NotImplementedError('cannot set_jvel in Gazebo')

        if joint_name is None:
            if len(velocity) != self.arm_dof:
                raise ValueError('Velocity should contain %d elements'
                                 'if the joint name is not '
                                 'provided' % self.arm_dof)
            tgt_vel = velocity
        else:
            if not isinstance(velocity, float):
                raise TypeError('velocity should be individual float value'
                                'if joint_name is provided')
            if joint_name not in self.arm_jnt_names_set:
                raise TypeError('Joint name [%s] is not in the arm'
                                ' joint list!' % joint_name)
            else:
                tgt_vel = [0.0] * len(self.arm_jnt_names)
                arm_jnt_idx = self.arm_jnt_names.index(joint_name)
                tgt_vel[arm_jnt_idx] = velocity
        if self.use_urscript:
            prog = 'speedj([%f, %f, %f, %f, %f, %f],' \
                   ' a=%f)' % (tgt_vel[0],
                               tgt_vel[1],
                               tgt_vel[2],
                               tgt_vel[3],
                               tgt_vel[4],
                               tgt_vel[5],
                               self._motion_acc)
            self._send_urscript(prog)
        else:
            # self._pub_joint_vel(tgt_vel)
            raise NotImplementedError('set_jvel only works '
                                      'with use_urscript=True')

        if wait:
            success = wait_to_reach_jnt_goal(
                velocity,
                get_func=self.get_jvel,
                joint_name=joint_name,
                timeout=self.cfgs.ARM.TIMEOUT_LIMIT,
                max_error=self.cfgs.ARM.MAX_JOINT_VEL_ERROR
            )

        return success

    def set_ee_pose(self, pos=None, ori=None, wait=True,
                    ik_first=False, *args, **kwargs):
        """
        Set cartesian space pose of end effector

        Args:
            pos (list or np.ndarray): Desired x, y, z positions in the robot's
                base frame to move to (shape: :math:`[3,]`)
            ori (list or np.ndarray, optional): It can be euler angles
                ([roll, pitch, yaw], shape: :math:`[4,]`),
                or quaternion ([qx, qy, qz, qw], shape: :math:`[4,]`),
                or rotation matrix (shape: :math:`[3, 3]`). If it's None,
                the solver will use the current end effector
                orientation as the target orientation
            wait (bool): wait until the motion completes
            ik_first (bool, optional): Whether to use the solution computed
                by IK, or to use UR built in movel function which moves
                linearly in tool space (movel may sometimes fail due to
                sinularities). This parameter takes effect only when
                self.use_urscript is True. Defaults to False.

        Returns:
            bool: Returns True is robot successfully moves to goal pose
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

        if self.use_urscript:
            if ik_first:
                jnt_pos = self.compute_ik(pos, quat)
                # use movej instead of movel
                success = self.set_jpos(jnt_pos, wait=wait)
            else:
                rotvec = arutil.quat2rotvec(quat)
                ee_pos = [pos[0], pos[1], pos[2],
                          rotvec[0], rotvec[1], rotvec[2]]
                # movel moves linear in tool space, may go through singularity
                # orientation in movel is specified as a rotation vector!
                # not euler angles!
                prog = 'movel(p[%f, %f, %f, %f, %f, %f], a=%f, v=%f, r=%f)' % (
                    ee_pos[0],
                    ee_pos[1],
                    ee_pos[2],
                    ee_pos[3],
                    ee_pos[4],
                    ee_pos[5],
                    self._motion_acc,
                    self._motion_vel,
                    0.0)
                self._send_urscript(prog)
                if wait:
                    args_dict = {
                        'get_func': self.get_ee_pose,
                        'get_func_derv': self.get_ee_vel,
                        'timeout': self.cfgs.ARM.TIMEOUT_LIMIT,
                        'pos_tol': self.cfgs.ARM.MAX_EE_POS_ERROR,
                        'ori_tol': self.cfgs.ARM.MAX_EE_ORI_ERROR
                    }
                    success = wait_to_reach_ee_goal(pos, quat,
                                                    **args_dict)
        else:
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
        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, wait=True,
                    *args, **kwargs):
        """
        Move end effector in straight line while maintaining orientation

        Args:
            delta_xyz (list or np.ndarray): Goal change in x, y, z position of
                end effector
            eef_step (float, optional): Discretization step in cartesian space
                for computing waypoints along the path. Defaults to 0.005 (m).
            wait (bool, optional): True if robot should not do anything else
                until this goal is reached, or the robot times out.
                Defaults to True.

        Returns:
            bool: True if robot successfully reached the goal pose
        """
        ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()

        if self.use_urscript:
            ee_pos[0] += delta_xyz[0]
            ee_pos[1] += delta_xyz[1]
            ee_pos[2] += delta_xyz[2]
            success = self.set_ee_pose(ee_pos, ee_euler, wait=wait,
                                       ik_first=False)
        else:
            plan = moveit_cartesian_path(ee_pos,
                                         ee_quat,
                                         delta_xyz,
                                         self.moveit_group,
                                         eef_step)
            success = self.moveit_group.execute(plan, wait=wait)
        return success

    def _init_ur_consts(self):
        """
        Initialize constants
        """

        self.gripper_tip_pos, self.gripper_tip_ori = self._get_tip_transform()

        self.scale_motion(vel_scale=0.2, acc_scale=0.2)

        # add a virtual base support frame of the real robot:
        ur_base_name = 'ur_base'
        ur_base_attached = False
        for _ in range(2):
            if self.moveit_scene.add_static_obj(
                    ur_base_name,
                    [0, 0, -0.5],
                    [0, 0, 0, 1],
                    size=[0.25, 0.50, 1.0],
                    obj_type='box',
                    ref_frame=self.cfgs.ARM.ROBOT_BASE_FRAME):
                ur_base_attached = True
                break
            time.sleep(1)
        if not ur_base_attached:
            print_red('Fail to add the UR base support as a collision object. '
                      'Be careful when you use moveit to plan the path! You '
                      'can try again to add the base manually.')

        if self.has_wrist_cam:
            # add a virtual bounding box for the wrist mounted camera
            wrist_cam_name = 'wrist_cam'
            wrist_cam_attached = False
            safe_camera_links = [
                'wrist_3_link',
                'ee_link',
                'robotiq_arg2f_coupling',
                'robotiq_arg2f_base_link'
            ]
            for _ in range(2):
                if self.moveit_scene.add_dynamic_obj(
                        'robotiq_arg2f_base_link',
                        wrist_cam_name,
                        [0.06, 0, 0.05],
                        [0, 0, 0, 1],
                        [0.03, 0.1, 0.03],
                        touch_links=safe_camera_links):
                    wrist_cam_attached = True
                    break
                time.sleep(1)
            if not wrist_cam_attached:
                print_red('Fail to add the wrist camera bounding box as collision'
                          'object. Be careful when you use moveit to plan paths!'
                          'You can try again to add the camera box manually.')

        # https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
        self._max_torques = [150, 150, 150, 28, 28, 28]

    def _send_urscript(self, prog):
        """
        Method to send URScript program to the URScript ROS topic

        Args:
            prog (str): URScript program which will be sent and run on
                the UR5e machine

        """
        # TODO return the status info
        # such as if the robot gives any error,
        # the execution is successful or not

        self.urscript_pub.publish(prog)

    def _output_pendant_msg(self, msg):
        """
        Method to display a text message on the UR5e teach pendant

        Args:
            msg (str): message to display

        """
        prog = 'textmsg(%s)' % msg
        self._send_urscript(prog)

    def _get_tip_transform(self):
        """
        Internal method to get the transform between the robot's
        wrist and the tip of the gripper

        Returns:
            2-element tuple containing

            - list: Translation component of the gripper tip transform
              (shape :math:`[3,]`)
            - list: Euler angle orientation component of the gripper
              tip transform. (shape :math:`[3,]`)
        """
        ee_frame = self.cfgs.ARM.ROBOT_EE_FRAME
        gripper_tip_id = self.arm_link_names.index(ee_frame)
        gripper_tip_link = self.urdf_chain.getSegment(gripper_tip_id)
        gripper_tip_tf = kdl_frame_to_numpy(gripper_tip_link.getFrameToTip())
        gripper_tip_pos = gripper_tip_tf[:3, 3].flatten()
        gripper_tip_rot_mat = gripper_tip_tf[:3, :3]
        gripper_tip_euler = arutil.rot2euler(gripper_tip_rot_mat)
        return list(gripper_tip_pos), list(gripper_tip_euler)

    def _set_tool_offset(self):
        """
        Internal method to send a URScript command to the robot so that
        it updates it tool center point variable to match the URDF
        """
        tool_offset_prog = 'set_tcp(p[%f, %f, %f, %f, %f, %f])' % (
            self.gripper_tip_pos[0],
            self.gripper_tip_pos[1],
            self.gripper_tip_pos[2],
            self.gripper_tip_ori[0],
            self.gripper_tip_ori[1],
            self.gripper_tip_ori[2]
        )

        self._output_pendant_msg(tool_offset_prog)
        self._send_urscript(tool_offset_prog)

    def _pub_joint_vel(self, velocity):
        """
        Received joint velocity command, puts it in a ROS trajectory
        msg, and uses internal publisher to send to /joint_speed topic
        on the real robot

        Args:
            velocity (list): List of desired joint velocities
                (length and order have already been checked)
        """
        goal_speed_msg = JointTrajectory()
        goal_speed_msg.points.append(
            JointTrajectoryPoint(
                velocities=velocity))
        self.joint_vel_pub.publish(goal_speed_msg)

    def _setup_pub_sub(self):
        """
        Initialize all the publishers and subscribers used internally
        """
        # for publishing joint speed to real robot
        self.joint_vel_pub = rospy.Publisher(
            self.cfgs.ARM.JOINT_SPEED_TOPIC,
            JointTrajectory,
            queue_size=2
        )

        self.urscript_pub = rospy.Publisher(
            self.cfgs.ARM.URSCRIPT_TOPIC,
            String,
            queue_size=10
        )
