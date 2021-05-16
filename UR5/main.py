import numpy as np
import os

from utils import combine, DEMONSTRATION_FILENAME
from Robot import Robot
from Demonstrate import Demonstrate
from DMP.DMP import DMP
from ServoControl import ServoControl
from FTSensor import FTSensor



if __name__ == '__main__':
    ip = "192.168.1.111"
    robot = Robot(ip)
    
    homeQ = [-0.3917191664325159, -1.7317592106261195, 1.9507082144366663, -1.7880441151061, -1.5626705328570765, -1.62141088]
    scaling = 1
    contactPoints = forceVectors = allPoints = []
    key = ''

    servoControl = ServoControl(robot)
    ft = FTSensor(robot)
    demonstrate = Demonstrate(robot)

    demonstrationFileExits = os.path.exists(DEMONSTRATION_FILENAME)
    if demonstrationFileExits:
        dmp = DMP()
    
    
    while True:
        print("h: Homes the robot.\nd: Records a demonstration.\ng: Set new goal for demonstration\ns: Set new start point for demonstration\nr: Run demonstration\nt: Set scaling on time constant (scaling: " + str(scaling) + ").\nn: Record noise on sensors.\np: Plot DMP.\nc: Clear obstacles.")
        try:
            key = input("Press a key ... then press enter: ")[0].lower()
        except:
            pass
        

        if key == 'h':
            print("\n\nHoming robot \n\n")
            robot.moveJ(homeQ)
        elif key == 'd':
            sec = 8
            print("Demonstrate started! Please teach it now :D. You have",sec,"seconds.")
            demonstrate.show(sec)
            dmp = DMP()
            demonstrationFileExits = True
        elif key == 'g':
            if not demonstrationFileExits:
                print("Can't set goal when no demonstration exits!")
            else:
                robot.teachMode()
                input("Move robot to new goal ... then press enter!")
                robot.endTeachMode()
                goal = robot.getActualTCPPose()[0:3]
                dmp.setGoal(goal)
                print("New goal set to",goal)
        elif key == 's':
            if not demonstrationFileExits:
                print("Can't set start when no demonstration exits!")
            else:
                robot.teachMode()
                input("Move robot to new start ... then press enter!")
                robot.endTeachMode()
                start = robot.getActualTCPPose()[0:3]
                dmp.setStart(start)
                print("New start set to",start)
        elif key == "r":
            dmp_p, dmp_dp, dmp_dpp = dmp.rollout(scaling)
            traj = combine(dmp_p)
            backtrackedTraj = servoControl.run(traj)
            if backtrackedTraj:
                contactPoints, forceVectors, allPoints = servoControl.searchForObstacle(backtrackedTraj)
                for point in contactPoints.T.tolist():
                    dmp.setObstacle(point)
        
        elif key == 'p':
            dmp_p, dmp_dp, dmp_dpp = dmp.rollout(scaling)
            dmp.plotTrajectory(np.asarray(forceVectors), np.asarray(contactPoints), np.asarray(allPoints))
            dmp.plotTrajectory(np.asarray([]), np.asarray([]), np.asarray([]), drawObstacles = True)

        elif key == 't':
            scaling = float(input("Set scaling on tau: "))
        
        elif key == 'n':
            ft.noiseFT(2)

        elif key == 'c':
            dmp.clearObstacles()
            contactPoints = forceVectors = allPoints = []
        elif key == 'f':
            robot.teachMode()
            input("Press enter to exit freedrive!")
            robot.endTeachMode()
        else:
            print("Please input a single valied char!")