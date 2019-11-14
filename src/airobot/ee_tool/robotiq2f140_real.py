import rospy
import threading
import struct
import socket
import time

from control_msgs.msg import GripperCommandActionGoal
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from airobot.ee_tool.ee import EndEffectorTool
from airobot.utils.common import clamp
from airobot.utils.urscript_util import Robotiq2F140URScript


class Robotiq2F140Real(EndEffectorTool):
    """
    Class for interfacing with Robotiq 2F140 gripper when
    it is attached to UR5e arm. Communication with the gripper
    is either through ROS over through a TCP/IP socket
    """

    def __init__(self, cfgs):
        """
        Constructor for Robotiq2F140 class

        Args:
            cfgs (YACS CfgNode): configurations for the gripper
        """
        super(Robotiq2F140Real, self).__init__(cfgs=cfgs)
        self.jnt_names = [
            'finger_joint', 'left_inner_knuckle_joint',
            'left_inner_finger_joint', 'right_outer_knuckle_joint',
            'right_inner_knuckle_joint', 'right_inner_finger_joint'
        ]

        self.gazebo_sim = rospy.get_param('sim')

        self.jnt_names_set = set(self.jnt_names)

        self._comm_initialized = False
        self._initialize_comm()

        self._gripper_data = None
        self._get_state_lock = threading.RLock()

    def activate(self):
        """
        Method to activate the gripper
        """
        if not self.gazebo_sim:
            urscript = self._get_new_urscript()

            urscript.set_activate()
            urscript.set_gripper_speed(self.cfgs.EETOOL.DEFAULT_SPEED)

            urscript.sleep(0.1)

            self.pub_command.publish(urscript())
        rospy.sleep(0.2)

    def set_pos(self, pos):
        """
        Set the gripper position. Function internally maps
        values from API position range to URScript position
        range

        Args:
            pos (float): Desired gripper position
        """
        pos = clamp(
            pos,
            self.cfgs.EETOOL.OPEN_ANGLE,
            self.cfgs.EETOOL.CLOSE_ANGLE
        )
        if not self.gazebo_sim:
            urscript = self._get_new_urscript()

            pos = int(pos * self.cfgs.EETOOL.POSITION_SCALING)

            urscript.set_gripper_position(pos)
            urscript.sleep(0.1)

            gripper_cmd = urscript()
        else:
            gripper_cmd = GripperCommandActionGoal()
            gripper_cmd.goal.command.position = pos
        self.pub_command.publish(gripper_cmd)

    def set_speed(self, speed):
        """
        Set the default speed which the gripper should move at

        Args:
            speed (int): Desired gripper speed (0 min, 255 max)
        """
        speed = int(clamp(speed, 0, 255))
        if not self.gazebo_sim:
            urscript = self._get_new_urscript()

            urscript.set_gripper_speed(speed)
            urscript.sleep(0.1)

            self.pub_command.publish(urscript())

    def open(self):
        """
        Open gripper
        """
        self.set_pos(self.cfgs.EETOOL.OPEN_ANGLE)

    def close(self):
        """
        Close gripper
        """
        self.set_pos(self.cfgs.EETOOL.CLOSE_ANGLE)

    def get_pos(self):
        """
        Get the current position of the gripper
        """
        self._get_state_lock.acquire()
        pos = self._gripper_data
        self._get_state_lock.release()
        return pos

    def _get_current_pos_cb(self, msg):
        """
        Callback for rospy subscriber to get joint information

        Args:
            msg (JointState): Contains the full joint state topic
                published
        """
        if 'finger_joint' in msg.name:
            idx = msg.name.index('finger_joint')
            if idx < len(msg.position):
                self._gripper_data = msg.position[idx]

    def _get_gripper_state(self):
        """
        Function to be run in background thread which creates socket
        connection with UR and receives position data
        """
        buffer_size = 1024
        state_socket = socket.socket(socket.AF_INET,
                                     socket.SOCK_STREAM)
        state_socket.bind((self.hostname, self.tcp_port))
        while True:
            time.sleep(0.005)
            state_socket.listen(1)
            conn = state_socket.accept()[0]
            data = conn.recv(buffer_size)
            if not data:
                continue
            self._gripper_data = int(struct.unpack('!i', data[0:4])[0])

    def _get_new_urscript(self):
        """
        Internal method used to create an empty URScript
        program, which is filled with URScript commands and
        eventually sent to the robot over one of the communication
        interfaces
        """
        urscript = Robotiq2F140URScript(
            socket_host=self.cfgs.EETOOL.SOCKET_HOST,
            socket_port=self.cfgs.EETOOL.SOCKET_PORT,
            socket_name=self.cfgs.EETOOL.SOCKET_NAME)

        urscript.sleep(0.1)
        return urscript

    def _initialize_comm(self):
        """
        Set up the internal publisher to send gripper command
        URScript programs to the robot thorugh ROS
        """
        if self.gazebo_sim:
            self.pub_command = rospy.Publisher(
                self.cfgs.EETOOL.GAZEBO_COMMAND_TOPIC,
                GripperCommandActionGoal,
                queue_size=10)
        else:
            self.pub_command = rospy.Publisher(
                self.cfgs.EETOOL.COMMAND_TOPIC,
                String,
                queue_size=10)
        self.sub_position = rospy.Subscriber(
            self.cfgs.EETOOL.JOINT_STATE_TOPIC,
            JointState,
            self._get_current_pos_cb
        )
        self._comm_initialized = True
