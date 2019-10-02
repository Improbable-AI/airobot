from airobot.end_effectors.ee import EndEffector


class Robotiq2F140(Camera):
    def __init__(self, cfgs, tcp_monitor, use_tcp=False):
        super(Robotiq2F140, self).__init__(cfgs=cfgs)
        self.tcp_monitor = tcp_monitor
        self.use_tcp = use_tcp
        self._tcp_initialized = False
        self._ros_initialized = False
        self.set_comm_mode(self.use_tcp)
        self._initialize_ros_comm()
        self._initialize_tcp_comm()

    def __del__(self):
        self.close_tcp()

    def open(self):
        if self.use_tcp:
            pass
        else:
            pass

    def close(self):
        if self.use_tcp:
            pass

    def close_tcp(self):
        if self._tcp_initialized:
            # close the tcp communication
            pass

    def _initialize_ros_comm(self):
        self._ros_initialized = True
        pass

    def _initialize_tcp_comm(self):
        self._tcp_initialized = True
        pass

    def set_comm_mode(self, tcp=False):
        self.use_tcp = tcp
