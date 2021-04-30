
import time
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface
import numpy as np
from scipy.spatial.transform import Rotation as R


def writeDatFile(contents,filename):
    with open(filename,'a') as file:
        for content in contents:
            string = ''
            for item in content:
                string += str(item) + " "
            string = string[0:-2]
            string += "\n"
            file.write(string)

class Robot(RTDEControlInterface, RTDEIOInterface, RTDEReceiveInterface):
    def __init__(self, ip):
        RTDEControlInterface.__init__(self, ip)
        RTDEReceiveInterface.__init__(self, ip, [], True)
        RTDEIOInterface.__init__(self, ip, 30004, True)

ip = "192.168.1.111"
robot = Robot(ip)

start = time.time()
currentTime = start
robot.teachMode()
demonstration = []
while currentTime - start < 5:
    currentTime = time.time()
    print("Time:",currentTime-start)
    demonstration.append(robot.getActualTCPPose())
    time.sleep(0.002)
    
robot.endTeachMode()
writeDatFile(demonstration, "demonstration.dat")