import socket
import struct
import threading
import time
from subprocess import check_output

import numpy as np
import rospy
from control_msgs.msg import GripperCommandActionGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from airobot.ee_tool.ee import EndEffectorTool
from airobot.utils.common import clamp
from airobot.utils.common import print_red
from airobot.utils.urscript_util import Robotiq2F140URScript


class Robotiq2F140Real(EndEffectorTool):
    """
    Class for interfacing with Robotiq 2F140 gripper when
    it is attached to UR5e arm. Communication with the gripper
    is either through ROS over through a TCP/IP socket.

    Args:
        cfgs (YACS CfgNode): configurations for the gripper.

    Attributes:
        cfgs (YACS CfgNode): configurations for the gripper.
        jnt_names (list): list of joint names of the gripper.
    """

    def __init__(self, cfgs):
        super(Robotiq2F140Real, self).__init__(cfgs=cfgs)
        self.jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint'
        ]

        self._gazebo_sim = rospy.get_param('sim')
        self._comm_initialized = False
        self._get_state_lock = threading.RLock()
        self._initialize_comm()

        if not self._gazebo_sim:
            self._gripper_data = None
            self._pub_state_lock = threading.RLock()

            self._updated_gripper_pos = JointState()
            self._updated_gripper_pos.name = ['finger_joint']
            self._updated_gripper_pos.position = [0.0]
            self._err_thresh = 1

            self._local_ip_addr = None
            local_ip = self._get_local_ip()
            # we assume the machine is connected to a router
            if local_ip is not None:
                self._local_ip_addr = local_ip
            else:
                raise ValueError('Could not get local ip address')

            self._get_current_pos_urscript()

            self._pub_gripper_thread = threading.Thread(
                target=self._pub_pos_target)
            self._pub_gripper_thread.daemon = True
            self._pub_gripper_thread.start()

    def activate(self):
        """
        Method to activate the gripper.
        """
        if not self._gazebo_sim:
            urscript = self._get_new_urscript()

            urscript.set_activate()
            urscript.set_gripper_speed(self.cfgs.EETOOL.DEFAULT_SPEED)

            urscript.sleep(0.1)

            self._pub_command.publish(urscript())
        time.sleep(3)
        if not self._gazebo_sim:
            self._get_current_pos_urscript()

    def set_jpos(self, pos):
        """
        Set the gripper position. Function internally maps
        values from API position range to URScript position
        range. After sending position command, update internal
        position variable by sending urscript program to
        controller.

        Args:
            pos (float): Desired gripper position.
        """
        pos = clamp(
            pos,
            self.cfgs.EETOOL.OPEN_ANGLE,
            self.cfgs.EETOOL.CLOSE_ANGLE
        )
        if not self._gazebo_sim:
            urscript = self._get_new_urscript()

            pos = int(pos * self.cfgs.EETOOL.POSITION_SCALING)

            urscript.set_gripper_position(pos)
            urscript.sleep(0.1)

            gripper_cmd = urscript()
        else:
            gripper_cmd = GripperCommandActionGoal()
            gripper_cmd.goal.command.position = pos

        self._pub_command.publish(gripper_cmd)
        time.sleep(1.0)
        if not self._gazebo_sim:
            self._get_current_pos_urscript()

    def set_speed(self, speed):
        """
        Set the default speed which the gripper should move at.

        Args:
            speed (int): Desired gripper speed (0 min, 255 max).
        """
        speed = int(clamp(speed, 0, 255))
        if not self._gazebo_sim:
            urscript = self._get_new_urscript()

            urscript.set_gripper_speed(speed)
            urscript.sleep(0.1)

            self._pub_command.publish(urscript())

    def open(self):
        """
        Open gripper.
        """
        self.set_jpos(self.cfgs.EETOOL.OPEN_ANGLE)

    def close(self):
        """
        Close gripper.
        """
        self.set_jpos(self.cfgs.EETOOL.CLOSE_ANGLE)

    def get_pos(self):
        """
        Get the current position of the gripper.
        """
        self._get_state_lock.acquire()
        pos = self._gripper_data
        self._get_state_lock.release()
        return pos

    def _get_current_pos_cb(self, msg):
        """
        Callback for rospy subscriber to get joint information.

        Args:
            msg (JointState): Contains the full joint state topic
                published.
        """
        if 'finger_joint' in msg.name:
            idx = msg.name.index('finger_joint')
            if idx < len(msg.position):
                self._get_state_lock.acquire()
                self._gripper_data = msg.position[idx]
                self._get_state_lock.release()

    def _get_new_urscript(self):
        """
        Internal method used to create an empty URScript
        program, which is filled with URScript commands and
        eventually sent to the robot over one of the communication
        interfaces.
        """
        urscript = Robotiq2F140URScript(
            socket_host=self.cfgs.EETOOL.SOCKET_HOST,
            socket_port=self.cfgs.EETOOL.SOCKET_PORT,
            socket_name=self.cfgs.EETOOL.SOCKET_NAME)

        urscript.sleep(0.1)
        return urscript

    def _get_current_pos_urscript(self):
        """
        Function to send a urscript message to the robot to update
        the gripper position value. URScript program is created to
        create socket connection with the remote machine, and the
        corresponding local socket is created to receive the incoming
        data. The value only updates after the gripper has stopped
        moving, by checking to see if the same received value is
        consecutively consistent, and is eventually published to the
        gripper state topic. Function will exit if timeout is reached.
        """
        if self._gazebo_sim:
            return
        tcp_port = 50201

        tcp_msg = 'def process():\n'
        tcp_msg += ' socket_open("127.0.0.1",63352,"gripper_socket")\n'
        tcp_msg += ' rq_pos = socket_get_var("POS","gripper_socket")\n'
        tcp_msg += ' sync()\n'
        tcp_msg += ' textmsg("value = ",rq_pos)\n'
        tcp_msg += ' socket_open("%s",%d,"desktop_socket")\n' % \
                   (self._local_ip_addr, tcp_port)
        tcp_msg += ' socket_send_int(rq_pos,"desktop_socket")\n'
        tcp_msg += ' sync()\n'
        tcp_msg += ' socket_close("desktop_socket")\n'
        tcp_msg += 'end\n'
        self._pub_command.publish(tcp_msg)

        returned_pos = None
        last_returned_pos = 0.0
        gripper_stopped = False
        check_equal_pos = 10
        equal_pos = 0

        start = time.time()

        hostname = socket.gethostbyname('0.0.0.0')
        buffer_size = 1024
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.settimeout(self.cfgs.EETOOL.UPDATE_TIMEOUT)
        s.bind((hostname, tcp_port))

        while returned_pos is None and not gripper_stopped:
            if time.time() - start > self.cfgs.EETOOL.UPDATE_TIMEOUT:
                prnt_str = 'Unable to update gripper position value in %f' \
                           ' s, exiting' % self.cfgs.EETOOL.UPDATE_TIMEOUT
                print_red(prnt_str)
                s.close()
                return
            try:
                s.listen(1)
                conn, _ = s.accept()
            except socket.timeout:
                prnt_str = 'Unable to accept from socket in %f' \
                           ' s, exiting' % self.cfgs.EETOOL.UPDATE_TIMEOUT
                print_red(prnt_str)
                s.close()
                return

            data = conn.recv(buffer_size)
            if not data:
                continue
            returned_pos = int(struct.unpack('!i', data[0:4])[0])

            if np.abs(returned_pos - last_returned_pos) < self._err_thresh:
                equal_pos += 1
            else:
                equal_pos = 0

            if equal_pos >= check_equal_pos:
                gripper_stopped = True

            last_returned_pos = returned_pos

        self._pub_state_lock.acquire()
        self._updated_gripper_pos.position[0] = returned_pos
        self._pub_state_lock.release()

        s.close()

    def _pub_pos_target(self):
        """
        Function to run in background thread to publish updated
        gripper state.
        """
        while not rospy.is_shutdown():
            try:
                self._pub_state_lock.acquire()
                self._pub_gripper_pos.publish(self._updated_gripper_pos)
                self._pub_state_lock.release()
                time.sleep(0.002)
            except rospy.ROSException:
                pass

    def _get_local_ip(self):
        """
        Function to get machine ip address on local network.

        Returns:
            str: Local IP address
        """
        raw_ips = check_output(['hostname', '--all-ip-addresses'])
        ips = raw_ips.decode('utf8')
        ip_list = ips.split()
        for ip in ip_list:
            if ip.startswith(self.cfgs.EETOOL.IP_PREFIX):
                return ip
        return self.cfgs.EETOOL.FULL_IP

    def _initialize_comm(self):
        """
        Set up the internal publisher to send gripper command
        URScript programs to the robot thorugh ROS.
        """
        if self._gazebo_sim:
            self._pub_command = rospy.Publisher(
                self.cfgs.EETOOL.GAZEBO_COMMAND_TOPIC,
                GripperCommandActionGoal,
                queue_size=10)
        else:
            self._pub_command = rospy.Publisher(
                self.cfgs.EETOOL.COMMAND_TOPIC,
                String,
                queue_size=10)
            self._pub_gripper_pos = rospy.Publisher(
                '/gripper_state',
                JointState,
                queue_size=10)
        self._sub_position = rospy.Subscriber(
            self.cfgs.EETOOL.JOINT_STATE_TOPIC,
            JointState,
            self._get_current_pos_cb
        )
        time.sleep(1.0)
        self._comm_initialized = True
