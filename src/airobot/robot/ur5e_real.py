"""
A UR5e robot with a robotiq 2f140 gripper
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import copy
import time
from abc import ABC

import PyKDL as kdl
import numpy as np
import rospy
from trac_ik_python import trac_ik
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_matrix

from airobot.robot.robot import Robot
from airobot.end_effectors.robotiq_gripper import Robotiq_2F140
from airobot.sensor.camera.rgbd_cam import RGBDCamera
from airobot.utils.tcp_util import SecondaryMonitor
from airobot.utils.common import clamp
from airobot.utils.common import joints_to_kdl
from airobot.utils.common import kdl_array_to_numpy
from airobot.utils.common import kdl_frame_to_numpy
from airobot.utils.common import print_red


class UR5eRobotReal(Robot):
    def __init__(self, cfgs, robot_ip, use_cam=False, use_arm=True,
                 moveit_planner='RRTConnectkConfigDefault'):
        try:
            rospy.init_node('ur5e', anonymous=True)
        except rospy.exceptions.ROSException:
            rospy.logwarn('ROS node [ur5e] has already been initialized')
        if use_cam:
            self.camera = RGBDCamera(cfgs=cfgs)
        if use_arm:
            super(UR5eRobotReal, self).__init__(cfgs=cfgs)
            self.moveit_planner = moveit_planner
            self.robot_ip = robot_ip
            self.monitor = SecondaryMonitor(self.robot_ip)
            self.monitor.wait()  # make contact with robot before anything
            self.gripper = Robotiq_2F140(montior=self.monitor,
                                         socker_host=self.cfgs.SOCKET_HOST,
                                         socker_port=self.cfgs.SOCKET_PORT)
            self._init_consts()

    def send_program(self, prog):
        """
        Method to send URScript program to the TCP/IP monitor

        Args:
            prog (str): URScript program which will be sent and run on
                the UR5e machine

        """
        self.monitor.send_program(prog)

    def output_pendant_msg(self, msg):
        """
        Method to display a text message on the UR5e teach pendant

        Args:
            msg (str): message to display

        Return:
            None
        """
        prog = 'textmsg(%s)' % msg
        self.send_program(prog)

    def _is_running(self):
        return self.monitor.running

    def go_home(self):
        """
        Move the robot to a pre-defined home pose
        """
        # 6 joints for the arm, 7th joint for the gripper
        self.set_jpos(self._home_position, wait=True)

    def set_gripper_pos(self, position):
        """
        Method to send a position command to the gripper
        by creating a URScript program which runs on the robot
        and forwards the control command to the gripper

        Args:
            position (float): Desired gripper position,
                0.0 is fully open, 0.7 is fully closed
        """
        position = clamp(position,
                         self.gripper_open_angle,
                         self.gripper_close_angle)
        urscript = self.gripper._get_new_urscript()
        urscript._set_gripper_position(position)
        self.monitor.send_program(urscript())

    def open_gripper(self):
        """
        Commands the gripper to fully open (0.0)
        """
        self.set_gripper_pos(self.gripper_open_angle)

    def close_gripper(self):
        """
        Commands the gripper to fully close (0.7)
        """
        self.set_gripper_pos(self.gripper_close_angle)

    def set_jpos(self, position, joint_name=None, wait=True, *args, **kwargs):
        """
        Method to send a joint position command to the robot

        Args:
            position (float or list): desired joint position(s)
            joint_name (str): If not provided, position should be a list and
                all actuated joints will be moved to specified positions. If
                provided, only specified joint will move
            wait (bool): whether position command should be blocking or non
                blocking

        Return:
            success (bool): whether command was completed successfully or not
        """
        position = copy.deepcopy(position)
        success = False

        if joint_name is None:
            if (len(position) != 6 or len(position != 7):
                raise ValueError('position should contain 6 or 7 elements if'
                                 'joint_name is not provided')
            if len(position) == 7:
                gripper_pos = position[-1]
                self.set_gripper_pos(gripper_pos)
                position = position[:-1]
            target_pos = position
        else:
            if joint_name is self.gripper_jnt_names:
                pass  # don't do anything with gripper yet
            else:
                target_pos_joint = position
                current_pos = self.get_jpos()
                rvl_jnt_idx = self.rvl_joint_names.index(joint_name)
                current_pos[rvl_jnt_idx] = target_pos_joint
                target_pos = copy.deepcopy(current_pos)
        prog = 'movej([%f, %f, %f, %f, %f, %f])' % (target_pos[0],
                                                    target_pos[1],
                                                    target_pos[2],
                                                    target_pos[3],
                                                    target_pos[4],
                                                    target_pos[5])
        self.send_program(prog)
        if wait:
            success = self._wait_to_reach_jnt_goal(target_pos,
                                                   joint_name=joint_name,
                                                   mode='pos')

        return success

    def set_jvel(self, velocity, acc=0.1, joint_name=None, wait=False,
                 *args, **kwargs):
        """
        Set joint velocity in rad/s

        Args:
            velocity (list): list of target joint velocity values
            joint_name (str, optional): If not provided, velocity should be
                list and all joints will be turned on at specified velocity.
                Defaults to None.
            wait (bool, optional): [description]. Defaults to False.
        """
        velocity = copy.deepcopy(velocity)
        success = False

        if joint_name is None:
            # if (len(velocity) == 6):
            #     gripper_vel = self.get_jvel(self.gripper_jnt_names[0])
            #     velocity.append(gripper_vel)

            if len(velocity) != 6:
                raise ValueError('Velocity should contain 6 or 7 elements'
                                 'if the joint name is not provided')

            target_vel = velocity

        else:
            if joint_name in self.gripper_jnt_names:
                pass
            else:
                target_vel_joint = velocity
                target_vel = [0.0] * 7
                rvl_jnt_idx = self.rvl_joint_names.index(joint_name)
                target_vel[rvl_jnt_idx] = target_vel_joint

        prog = 'speedj([%f, %f, %f, %f, %f, %f], a=%f)' % (target_vel[0],
                                                           target_vel[1],
                                                           target_vel[2],
                                                           target_vel[3],
                                                           target_vel[4],
                                                           target_vel[5],
                                                           acc)
        self.send_program(prog)

        if wait:
            success = self._wait_to_reach_jnt_goal(target_vel,
                                                   joint_name=joint_name,
                                                   mode='vel')

        return success

    def set_ee_pose(self, pos, ori=None, acc=0.1, vel=0.05, wait=True,
                    linear_path=False, *args, **kwargs):
        """
        Set cartesian space pose of end effector

        Args:
            pos (list): Desired x, y, z positions in the robot's base frame to
                move to
            ori (list, optional): Desired euler angle orientation (roll, pitch, yaw)
                or quaternion ([x, y, z, w]) of the end effector. It Defaults to None.
            acc (float, optional): Acceleration of end effector during
                beginning of movement. Defaults to 0.1.
            vel (float, optional): Velocity of end effector during movement.
                Defaults to 0.05.

        Returns:
            bool: success or failure to move the robot to the goal pose
        """
        success = False
        if ori is None:
            ori = self.get_ee_pose()[-1]  # last index of return is euler angle
        if len(ori == 4):
            # assume incoming orientation is quaternion
            ori = euler_from_quaternion(ori)
        ee_pos = [pos[0], pos[1], pos[2], ori[0], ori[1], ori[2]]
        if linear_path:
            prog = 'movel(p[%f, %f, %f, %f, %f, %f], a=%f, v=%f, r=%f)' % (
                ee_pos[0],
                ee_pos[1],
                ee_pos[2],
                ee_pos[3],
                ee_pos[4],
                ee_pos[5],
                acc,
                vel,
                0.0)
            self.send_program(prog)
        else:
            jnt_pos = self.compute_ik(pos, ori)  # ik can handle quaternion
            self.set_jpos(jnt_pos)

        if wait:
            if linear_path:
                success = self._wait_to_reach_ee_goal(ee_pos)
            else:
                success = self._wait_to_reach_jnt_goal(jnt_pos)

        return success

    def move_ee_xyz(self, delta_xyz, eef_step=0.005, wait=True,
                    linear_path=False, *args, **kwargs):
        """Move end effector in straight line while maintaining orientation

        Args:
            delta_xyz (list): Goal change in x, y, z position of end effector
            eef_step (float, optional): [description]. Defaults to 0.005.
        """
        success = True
        ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()

        if linear_path:
            ee_pos[0] += delta_xyz[0]
            ee_pos[1] += delta_xyz[1]
            ee_pos[2] += delta_xyz[2]

            success = self.set_ee_pose(ee_pos, ee_euler, wait=wait,
                                       linear_path=linear_path)
        else:
            current_pos = np.array(ee_pos)
            delta_xyz = np.array(delta_xyz)
            path_len = np.linalg.norm(delta_xyz)
            num_pts = int(np.ceil(path_len / float(eef_step)))
            if num_pts <= 1:
                num_pts = 2

            waypoints_sp = np.linspace(0, path_len, num_pts).reshape(-1, 1)
            waypoints = current_pos + waypoints_sp / float(path_len) * \
                        delta_xyz

            for i in range(waypoints.shape[0]):
                tgt_jnt_poss = \
                    self.set_ee_pose(waypoints[i, :].flatten().tolist(),
                                     ee_quat)
                self.set_jpos(tgt_jnt_poss)
                tgt_jnt_poss = np.array(tgt_jnt_poss)
                start_time = time.time()
                while True:
                    if (time.time() - start_time >
                            self.cfgs.TIMEOUT_LIMIT):
                        pt_str = "Unable to move to joint angles (%s)" \
                                 " within %f s" % (str(tgt_jnt_poss.tolist()),
                                                   self.cfgs.TIMEOUT_LIMIT)
                        print_red(pt_str)
                        success = False
                        return success
                    new_jnt_poss = self.get_jpos()
                    new_jnt_poss = np.array(new_jnt_poss)
                    jnt_diff = new_jnt_poss - tgt_jnt_poss
                    error = np.max(np.abs(jnt_diff))
                    if error < self.cfgs.MAX_JOINT_ERROR:
                        break
                time.sleep(0.001)
            success = True
        return success

    def get_jpos(self, joint_name=None):
        """Get current joint angles of robot

        Args:
            joint_name (str, optional): Defaults to None.

        Return:
            jpos (list): list of current joint positions in radians
        """
        jdata = self.monitor.get_joint_data()
        jpos = [jdata["q_actual0"], jdata["q_actual1"], jdata["q_actual2"],
                jdata["q_actual3"], jdata["q_actual4"], jdata["q_actual5"]]
        return jpos

    def get_jvel(self, joint_name=None):
        """Get current joint angular velocities of robot

        Args:
            joint_name (str, optional): Defaults to None.

        Return:
            jvel (list): list of current joint angular velocities in radians/s
        """
        jdata = self.monitor.get_joint_data()
        jvel = [jdata['qd_actual0'], jdata['qd_actual1'], jdata['qd_actual2'],
                jdata['qd_actual3'], jdata['qd_actual4'], jdata['qd_actual5']]
        return jvel

    def get_ee_pose(self):
        """Get current cartesian pose of the EE, in the robot's base frame

        Args:
            wait (bool, optional): [description]. Defaults to False.

        Returns:
            list: x, y, z position of the EE (shape: [3])
            list: quaternion representation ([x, y, z, w]) of the EE orientation (shape: [4])
            list: rotation matrix representation of the EE orientation
                (shape: [9])
            list: euler angle representation of the EE orientation (roll,
                pitch, yaw with static reference frame) (shape: [3])
        """
        pose_data = self.monitor.get_cartesian_info()
        if pose_data:
            pos = [pose_data["X"], pose_data["Y"], pose_data["Z"]]
            euler_ori = [pose_data["Rx"], pose_data["Ry"], pose_data["Rz"]]
            rot_mat = euler_matrix(*euler_ori)[:3, :3].tolist()
            quat_ori = quaternion_from_euler(*euler_ori).tolist()
        else:
            raise RuntimeError('Cannot get pose information!')
        return pos, quat_ori, rot_mat, euler_ori

    def get_images(self, get_rgb=True, get_depth=True, **kwargs):
        """
        Return rgba/depth images

        Args:
            get_rgb (bool): return rgb image if True, None otherwise
            get_depth (bool): return depth image if True, None otherwise

        Returns:
            np.ndarray: rgba and depth images
        """
        return self.camera.get_images(get_rgb, get_depth)

    def get_jacobian(self, joint_angles):
        """
        Return the geometric jacobian on the given joint angles.
        Refer to P112 in "Robotics: Modeling, Planning, and Control"

        Args:
            joint_angles (list or flattened np.ndarray): joint angles

        Returns:
            jacobian (list, shape: [6, 6])
        """
        q = kdl.JntArray(self.urdf_chain.getNrOfJoints())
        for i in range(q.rows()):
            q[i] = joint_angles[i]
        jac = kdl.Jacobian(self.urdf_chain.getNrOfJoints())
        fg = self.jac_solver.JntToJac(q, jac)
        assert fg == 0, 'KDL JntToJac error!'
        jac_np = kdl_array_to_numpy(jac)
        return jac_np.tolist()

    def compute_fk_position(self, jpos, tgt_frame):
        """
        Given joint angles, compute the pose of desired_frame with respect
        to the base frame (self.cfgs.ROBOT_BASE_FRAME). The desired frame
        must be in self.arm_link_names

        Args:
            jpos (list or flattened np.ndarray): joint angles
            tgt_frame (str): target link frame

        Returns:
            translational vector (list, shape: [3,])
            and rotational matrix (list, shape: [3, 3])
        """
        if isinstance(jpos, list):
            jpos = np.array(jpos)
        jpos = jpos.flatten()
        if jpos.size != self.arm_dof:
            raise ValueError('Length of the joint angles '
                             'does not match the robot DOF')
        assert jpos.size == self.arm_dof
        kdl_jnt_angles = joints_to_kdl(jpos)

        kdl_end_frame = kdl.Frame()
        idx = self.arm_link_names.index(tgt_frame) + 1
        fg = self.fk_solver_pos.JntToCart(kdl_jnt_angles,
                                          kdl_end_frame,
                                          idx)
        if fg == 0:
            raise ValueError('KDL Pos JntToCart error!')
        pose = kdl_frame_to_numpy(kdl_end_frame)
        pos = pose[:3, 3].flatten().tolist()
        rot = pose[:3, :3].tolist()
        return pos, rot

    def compute_fk_velocity(self, jpos, jvel, tgt_frame):
        """
        Given joint_positions and joint velocities,
        compute the velocities of des_frame with respect
        to the base frame

        Args:
            jpos (list or flattened np.ndarray): joint positions
            jvel (list or flattened np.ndarray): joint velocities
            tgt_frame (str): target link frame

        Returns:
            translational and rotational
                 velocities (vx, vy, vz, wx, wy, wz)
                 (list, shape: [6,])
        """
        if isinstance(jpos, list):
            jpos = np.array(jpos)
        if isinstance(jvel, list):
            jvel = np.array(jvel)
        kdl_end_frame = kdl.FrameVel()
        kdl_jnt_angles = joints_to_kdl(jpos)
        kdl_jnt_vels = joints_to_kdl(jvel)
        kdl_jnt_qvels = kdl.JntArrayVel(kdl_jnt_angles, kdl_jnt_vels)
        idx = self.arm_link_names.index(tgt_frame) + 1
        fg = self.fk_solver_vel.JntToCart(kdl_jnt_qvels,
                                          kdl_end_frame,
                                          idx)
        if fg == 0:
            raise ValueError('KDL Vel JntToCart error!')
        end_twist = kdl_end_frame.GetTwist()
        return [end_twist[0], end_twist[1], end_twist[2],
                end_twist[3], end_twist[4], end_twist[5]]

    def compute_ik(self, pos, ori=None, qinit=None, *args, **kwargs):
        """
        Compute the inverse kinematics solution given the
        position and orientation of the end effector
        (self.cfgs.ROBOT_EE_FRAME)

        Args:
            pos (list): position
            ori (list): orientation. It can be euler angles
                (roll, pitch, yaw) or quaternion. If it's None,
                the solver will use the current end effector
                orientation as the target orientation
            qinit (list): initial joint positions for numerical IK

        Returns:
            inverse kinematics solution (joint angles, list)
        """
        if ori is not None:
            if len(ori) == 3:
                # [roll, pitch, yaw]
                ori = quaternion_from_euler(*ori)
            if len(ori) != 4:
                raise ValueError('Orientation should be either '
                                 'euler angles or quaternion')
            ori_x = ori[0]
            ori_y = ori[1]
            ori_z = ori[2]
            ori_w = ori[3]
        else:
            ee_pos, ee_quat, ee_rot_mat, ee_euler = self.get_ee_pose()
            ori_x = ee_quat[0]
            ori_y = ee_quat[1]
            ori_z = ee_quat[2]
            ori_w = ee_quat[3]
        if qinit is None:
            qinit = self.get_jpos().tolist()
        elif isinstance(qinit, np.ndarray):
            qinit = qinit.flatten().tolist()
        pos_tol = self.cfgs.IK_POSITION_TOLERANCE
        ori_tol = self.cfgs.IK_ORIENTATION_TOLERANCE
        jnt_poss = self.num_ik_solver.get_ik(qinit,
                                             pos[0],
                                             pos[1],
                                             pos[2],
                                             ori_x,
                                             ori_y,
                                             ori_z,
                                             ori_w,
                                             pos_tol,
                                             pos_tol,
                                             pos_tol,
                                             ori_tol,
                                             ori_tol,
                                             ori_tol)
        if jnt_poss is None:
            return None
        return list(jnt_poss)

    def _wait_to_reach_jnt_goal(self, goal, joint_name=None, mode='pos'):
        """
        Block the code to wait for the joint moving to the specified goal.
        The goal can be a desired velocity(s) or a desired position(s).
        Max waiting time is self.cfgs.TIMEOUT_LIMIT

        Args:
            goal (float or list): goal positions or velocities
            joint_name (str): if it's none, all the actuated
                joints are compared.
                Otherwise, only the specified joint is compared
            mode (str): 'pos' or 'vel'

        Returns:
            if the goal is reached or not
        """
        success = False
        start_time = time.time()
        while True:
            if not self._is_running():
                raise RuntimeError("Robot stopped")

            if time.time() - start_time > self.cfgs.TIMEOUT_LIMIT:
                pt_str = 'Unable to move to joint goals [mode: %s] (%s)' \
                         ' within %f s' % (mode, str(goal),
                                           self.cfgs.TIMEOUT_LIMIT)
                print_red(pt_str)
                return success
            if self._reach_jnt_goal(goal, joint_name, mode=mode):
                success = True
                break
            time.sleep(0.001)
        return success

    def _reach_jnt_goal(self, goal, joint_name=None, mode='pos'):
        """
        Check if the joint reached the goal or not.
        The goal can be a desired velocity(s) or a desired position(s).

        Args:
            goal (float or list): goal positions or velocities
            joint_name (str): if it's none, all the
                actuated joints are compared.
                Otherwise, only the specified joint is compared
            mode (str): 'pose' or 'vel'

        Returns:
            if the goal is reached or not
        """
        goal = np.array(goal)
        if mode == 'pos':
            new_jnt_val = self.get_jpos(joint_name)
        elif mode == 'vel':
            new_jnt_val = self.get_jvel(joint_name)
        else:
            raise ValueError('Only pos and vel modes are supported!')
        new_jnt_val = np.array(new_jnt_val)
        jnt_diff = new_jnt_val - goal
        error = np.max(np.abs(jnt_diff))
        if error < self.cfgs.MAX_JOINT_ERROR:
            return True
        else:
            return False

    def _wait_to_reach_ee_goal(self, goal):
        """
        Block the code to wait for the end effector to reach its
        specified goal pose (must be below both position and
        orientation threshold). Max waiting time is
        self.cfgs.TIMEOUT_LIMIT

        Args:
            goal (list): Goal pose for end effector

        Returns:
            bool: If end effector reached goal or not
        """
        success = False
        start_time = time.time()
        while True:
            if not self._is_running():
                raise RuntimeError("Robot stopped")

            if time.time() - start_time > self.cfgs.TIMEOUT_LIMIT:
                pt_str = 'Unable to move to end effector goal:' \
                         '%s within %f s' % (str(goal),
                                             self.cfgs.TIMEOUT_LIMIT)
                print_red(pt_str)
                return success
            if self._reach_ee_goal(goal):
                success = True
                break
            time.sleep(0.001)
        return success

    def _reach_ee_goal(self, goal):
        """
        Check if end effector reached goal or not. Returns true
        if both position and orientation goals have been reached
        within specified tolerance

        Args:
            goal (list): Goal pose (position and orientation) of the
                end effector

        Returns:
            bool: If goal pose is reached or not
        """
        goal_pos = np.array(goal[0:3])
        goal_ori = np.array(goal[3:])

        new_ee_pose = self.get_ee_pose()
        new_ee_pos = np.array(new_ee_pose[0])
        new_ee_ori = np.array(new_ee_pose[-1])

        pos_diff = new_ee_pos - goal_pos
        ori_diff = new_ee_ori - goal_ori

        pos_error = np.max(np.abs(pos_diff))
        ori_error = np.max(np.abs(ori_diff))

        if pos_error < self.cfgs.MAX_EE_POSITION_ERROR and \
                ori_error < self.cfgs.MAX_EE_ORIENTATION_ERROR:
            return True
        else:
            return False

    def _init_consts(self):
        """
        Initialize constants
        """
        self._home_position = self.cfgs.HOME_POSITION

        robot_description = self.cfgs.ROBOT_DESCRIPTION
        urdf_string = rospy.get_param(robot_description)
        self.num_ik_solver = trac_ik.IK(self.cfgs.ROBOT_BASE_FRAME,
                                        self.cfgs.ROBOT_EE_FRAME,
                                        urdf_string=urdf_string)
        _, self.urdf_tree = treeFromParam(robot_description)
        # TODO get base transform from gripper_tip, from urdf tree or chain
        self.urdf_chain = self.urdf_tree.getChain(self.cfgs.ROBOT_BASE_FRAME,
                                                  self.cfgs.ROBOT_EE_FRAME)
        self.arm_jnt_names = self._get_kdl_joint_names()
        self.arm_jnt_names_set = set(self.arm_jnt_names)
        self.arm_link_names = self._get_kdl_link_names()
        self.arm_dof = len(self.arm_joint_names)
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_group = MoveGroupCommander(self.cfgs.MOVEGROUP_NAME)
        self.moveit_group.set_planner_id(self.moveit_planner)
        self.moveit_scene = moveit_commander.PlanningSceneInterface()

        self.jac_solver = kdl.ChainJntToJacSolver(self.urdf_chain)
        self.fk_solver_pos = kdl.ChainFkSolverPos_recursive(self.urdf_chain)
        self.fk_solver_vel = kdl.ChainFkSolverVel_recursive(self.urdf_chain)

        self.gripper_jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint'
        ]
        self.gripper_jnt_names_set = set(self.gripper_jnt_names)
        self.gripper_close_angle = 0.7
        self.gripper_open_angle = 0

        self.ee_link = self.cfgs.ROBOT_EE_FRAME

        # https://www.universal-robots.com/how-tos-and-faqs/faq/ur-faq/max-joint-torques-17260/
        self._max_torques = [150, 150, 150, 28, 28, 28]
        # a random value for robotiq joints
        self._max_torques.append(20)

    def scale_moveit_motion(self, vel_scale=1.0, acc_scale=1.0):
        vel_scale = clamp(vel_scale, 0.0, 1.0)
        acc_scale = clamp(acc_scale, 0.0, 1.0)
        self.moveit_group.set_max_velocity_scaling_factor(vel_scale)
        self.moveit_group.set_max_acceleration_scaling_factor(acc_scale)

    def _get_kdl_link_names(self):
        num_links = self.urdf_chain.getNrOfSegments()
        link_names = []
        for i in range(num_links):
            link_names.append(self.urdf_chain.getSegment(i).getName())
        return copy.deepcopy(link_names)

    def _get_kdl_joint_names(self):
        num_links = self.urdf_chain.getNrOfSegments()
        num_joints = self.urdf_chain.getNrOfJoints()
        joint_names = []
        for i in range(num_links):
            link = self.urdf_chain.getSegment(i)
            joint = link.getJoint()
            joint_type = joint.getType()
            # JointType definition: [RotAxis,RotX,RotY,RotZ,TransAxis,
            #                        TransX,TransY,TransZ,None]
            if joint_type > 1:
                continue
            joint_names.append(joint.getName())
        assert num_joints == len(joint_names)
        return copy.deepcopy(joint_names)

    def _close(self):
        self.monitor.close()
