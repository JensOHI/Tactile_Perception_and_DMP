from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface
import numpy as np
from scipy.spatial.transform import Rotation as R


class Robot(RTDEControlInterface, RTDEIOInterface, RTDEReceiveInterface):
    def __init__(self, ip):
        RTDEControlInterface.__init__(self, ip, True, True)
        RTDEReceiveInterface.__init__(self, ip, [], True)
        RTDEIOInterface.__init__(self, ip, 30004, True)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    ip = "192.168.1.111"
    robot = Robot(ip)
    
    tcp_pose = np.array(robot.getActualTCPPose())
    tcp_position = tcp_pose[0:3]
    tcp_axis_angle = tcp_pose[3:6]
    rot = R.from_rotvec(tcp_axis_angle)
    r_matrix = rot.as_matrix().inv()
    print(r_matrix)
    # print(tcp_pose)
    robot.zeroFtSensor()
    input("press enter")
    tcp_ft = np.array(robot.getActualTCPForce())
    forces_in_base_rot = tcp_ft[0:3]
    torque_in_base_rot = tcp_ft[3:6]

    forces_in_tcp = r_matrix @ forces_in_base_rot
    torque_in_tcp = r_matrix @ torque_in_base_rot

    print("Force in base: ", forces_in_base_rot)
    print("Force in tcp: ", forces_in_tcp)






