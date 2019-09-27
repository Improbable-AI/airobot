import logging
import os

from urscript_util import URScript

# Gripper Variables
ACT = "ACT"
GTO = "GTO"
ATR = "ATR"
ARD = "ARD"
FOR = "FOR"
SPE = "SPE"
OBJ = "OBJ"
STA = "STA"
FLT = "FLT"
POS = "POS"


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
        self._socket_send_string("GET {}".format(var_name))
        self._socket_read_byte_list(nbytes)

    def _get_gripper_fault(self):
        self._rq_get_var(FLT, 2)

    def _get_gripper_object(self):
        self._rq_get_var(OBJ, 1)

    def _get_gripper_status(self):
        self._rq_get_var(STA, 1)

    def _set_gripper_activate(self):
        self._socket_set_var(GTO, 1, self.socket_name)

    def _set_gripper_force(self, value):
        """
        FOR is the variable
        range is 0 - 255
        0 is no force
        255 is full force
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(FOR, value, self.socket_name)

    def _set_gripper_position(self, value):
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(POS, value, self.socket_name)

    def _set_gripper_speed(self, value):
        """
        SPE is the variable
        range is 0 - 255
        0 is no speed
        255 is full speed
        """
        value = self._constrain_unsigned_char(value)
        self._socket_set_var(SPE, value, self.socket_name)

    def _set_robot_activate(self):
        self._socket_set_var(ACT, 1, self.socket_name)


class Robotiq_2F140(object):

    def __init__(self,
                 monitor,
                 socket_host,
                 socket_port,
                 socket_name="gripper_socket",
                 payload=0.85,
                 speed=255,
                 force=50):
        self.monitor = monitor
        self.payload = payload
        self.speed = speed
        self.force = force
        self.socket_host = socket_host
        self.socket_port = socket_port
        self.socket_name = socket_name
        self.logger = logging.getLogger(u"robotiq")

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
        urscript = self._get_new_urscript()

        urscript._set_gripper_force(self.force)
        urscript._set_gripper_speed(self.speed)

        urscript._set_robot_activate()
        urscript._set_gripper_activate()

        urscript._sleep(0.1)

        self.monitor.send_program(urscript())
