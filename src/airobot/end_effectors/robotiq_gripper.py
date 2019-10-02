import logging
import os

import rospy
from control_msgs.msg import GripperCommandActionGoal

from airobot.utils.urscript_util import URScript
from airobot.utils.common import clamp


# Gripper Variables
ACT = 'ACT'
GTO = 'GTO'
ATR = 'ATR'
ARD = 'ARD'
FOR = 'FOR'
SPE = 'SPE'
OBJ = 'OBJ'
STA = 'STA'
FLT = 'FLT'
POS = 'POS'


class RobotiqScript(URScript):

    def __init__(self,
                 socket_host,
                 socket_port,
                 socket_name):
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        super(RobotiqScript, self).__init__()

        # Reset connection to gripper
        self._socket_close(self.socket_name)
        self._socket_open(self.socket_host,
                          self.socket_port,
                          self.socket_name)

    def _import_rq_script(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        rq_script = os.path.join(dir_path, 'rq_script.script')
        with open(rq_script, 'rb') as f:
            rq_script = f.read()
            self.add_header_to_program(rq_script)

    def _rq_get_var(self, var_name, nbytes):
        self._socket_send_string('GET {}'.format(var_name))
        self._socket_read_byte_list(nbytes)

    def get_gripper_fault(self):
        self._rq_get_var(FLT, 2)

    def get_gripper_object(self):
        self._rq_get_var(OBJ, 1)

    def get_gripper_status(self):
        self._rq_get_var(STA, 1)

    def set_gripper_activate(self):
        self._socket_set_var(GTO, 1, self.socket_name)

    def set_gripper_force(self, value):
        """
        FOR is the variable
        range is 0 - 255
        0 is no force
        255 is full force
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(FOR, value, self.socket_name)

    def set_gripper_position(self, value):
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(POS, value, self.socket_name)

    def set_gripper_speed(self, value):
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(SPE, value, self.socket_name)

    def set_robot_activate(self):
        self._socket_set_var(ACT, 1, self.socket_name)


class Robotiq2F140(object):
    def __init__(self,
                 monitor,
                 socket_host,
                 socket_port,
                 open_angle,
                 close_angle,
                 socket_name='gripper_socket',
                 payload=0.85,
                 speed=255,
                 force=50,
                 output_range=255):
        self.monitor = monitor
        self.payload = payload
        self.speed = speed
        self.force = force
        self.open_angle = open_angle
        self.close_angle = close_angle
        self.input_range = close_angle - open_angle
        self.output_range = output_range
        self.pos_range_scaling = (self.output_range/self.input_range)
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        self.logger = logging.getLogger(u'robotiq')

    def _get_new_urscript(self):
        """
        Set up a new URScript to communicate with gripper
        """
        urscript = RobotiqScript(socket_host=self.socket_host,
                                 socket_port=self.socket_port,
                                 socket_name=self.socket_name)

        # Wait on activation to avoid USB conflicts
        urscript._sleep(0.1)

        return urscript

    def activate_gripper(self):
        """
        Sen URScript command to activate the gripper
        """
        urscript = self._get_new_urscript()

        urscript.set_gripper_force(self.force)
        urscript.set_gripper_speed(self.speed)

        urscript.set_robot_activate()
        urscript.set_gripper_activate()

        urscript._sleep(0.1)

        self.monitor.send_program(urscript())

    def set_gripper_pos(self, position):
        """
        Method to send a position command to the gripper
        by creating a URScript program which runs on the robot
        and forwards the control command to the gripper

        Args:
            position (float): Desired gripper position,
                fully open and closed position values specified
                upon construction
        """
        position = clamp(position,
                         self.open_angle,
                         self.close_angle)
        position = int(position * self.pos_range_scaling)
        position = min(position, self.output_range)

        urscript = self._get_new_urscript()
        urscript.set_gripper_position(position)
        self.monitor.send_program(urscript())

    def open_gripper(self):
        """
        Commands the gripper to fully open
        """
        self.set_gripper_pos(self.open_angle)

    def close_gripper(self):
        """
        Commands the gripper to fully close
        """
        self.set_gripper_pos(self.close_angle)


class Robotiq2F140Sim(object):
    def __init__(self,
                 topic,
                 open_angle,
                 close_angle,
                 output_range=0.7):
        self.gripper_topic = topic
        self.open_angle = open_angle
        self.close_angle = close_angle
        self.input_range = close_angle - open_angle
        self.output_range = output_range
        self.pos_range_scaling = (self.output_range/self.input_range)
        self.pub = rospy.Publisher(
            self.gripper_topic, GripperCommandActionGoal, queue_size=10)

    def set_gripper_pos(self, position):
        """
        Method to send a position command to the gripper
        by creating a ROS msg to send to the gripper controller

        Args:
            position (float): Desired gripper position,
                fully open and closed position values specified
                upon construction
        """
        gripper_cmd = GripperCommandActionGoal()
        gripper_cmd.goal = position
        self.pub.publish(gripper_cmd)

    def open_gripper(self):
        """
        Commands the gripper to fully open
        """
        self.set_gripper_pos(self.open_angle)

    def close_gripper(self):
        """
        Commands the gripper to fully close
        """
        self.set_gripper_pos(self.close_angle)
