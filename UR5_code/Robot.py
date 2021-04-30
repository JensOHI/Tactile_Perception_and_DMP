from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface

class Robot(RTDEControlInterface, RTDEIOInterface, RTDEReceiveInterface):
    def __init__(self, ip):
        RTDEControlInterface.__init__(self, ip)
        RTDEReceiveInterface.__init__(self, ip, [], True)
        RTDEIOInterface.__init__(self, ip, 30004, True)