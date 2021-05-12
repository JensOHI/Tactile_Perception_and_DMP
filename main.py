from numpy.ma.core import concatenate
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface
import numpy as np
#from scipy.spatial.transform import Rotation as R
from UR5_code.Robot import Robot
from UR5_code.Demonstrate import Demonstrate
from DMP.DMP import DMP
from UR5_code.ServoControl import ServoControl
from UR5_code.FTSensor.FTSensor import FTSensor
import numpy as np
import threading
import time
from UR5_code.utils import combine


def generateAObstacle(trajectory):
    pos = trajectory[round(len(trajectory)/2)][0:3]
    return pos + [0.05]

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    ip = "192.168.1.111"
    robot = Robot(ip)


    #print(robot.getActualTCPPose())
    #homePosition = [-0.4332957949604125, 0.03666119829241225, 0.3959256349973932, 2.3481883870878946, 2.087005826292801, 6.066972042799972e-05]
    homeQ = [-0.3917191664325159, -1.7317592106261195, 1.9507082144366663, -1.7880441151061, -1.5626705328570765, -1.62141088]
    robot.moveJ(homeQ)
    
    newGoal = [-0.8211983656828382, -0.05282045087594499, 0.07058759409960978, -0.5646979690491176, -3.0894219083737653, 0.026431204595942472]
    
    #input("Start demonstration! Press enter...")
    #demonstrate = Demonstrate(robot)
    #trajectory = demonstrate.show(8,saveAsFile=True)
    
    input("Start noise collection! Press enter...")
    ft = FTSensor(robot)
    ft.noiseFT(5)
    input("Start CUSUM. Press enter...")
    event = threading.Event()
    
    

    scaling = 1
    dmp = DMP()
    #dmp.setGoal(newGoal[0:3])
    #obstacle = generateAObstacle(trajectory)
    #print(obstacle)
    #dmp.setObstacle(obstacle)
    dmp_p, dmp_dp, dmp_dpp = dmp.rollout(scaling)

    
    traj = combine(dmp_p)
    #input("Play DMP. Press enter...")
    #robot.moveL(traj[-1])
    servoControl = ServoControl(robot)
     
    backtrackedTraj = servoControl.run(event, traj, scaling)
    
    
    if backtrackedTraj:
        contactPoints, forceVectors, allPoints = servoControl.searchForObstacle(event, backtrackedTraj)
        #dmp.plotTrajectory(forceVectors=forceVectors, contactPoints=contactPoints, allPoints=allPoints)

        for point in contactPoints.T.tolist():
            dmp.setObstacle(point)
        dmp_p, dmp_dp, dmp_dpp = dmp.rollout(scaling)
        dmp.plotTrajectory(forceVectors, contactPoints, allPoints)
        traj = combine(dmp_p)
        event.clear()
        backtrackedTraj = servoControl.run(event, traj, scaling)


    

    
    

    



    '''
    tcp_pose = np.array(robot.getActualTCPPose())
    tcp_position = tcp_pose[0:3]
    tcp_axis_angle = tcp_pose[3:6]
    rot = R.from_rotvec(tcp_axis_angle)
    r_matrix = rot.as_matrix()
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
    '''






