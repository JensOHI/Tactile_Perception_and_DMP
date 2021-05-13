import time
import threading
from .FTSensor.FTSensor import FTSensor
import numpy as np
import scipy
from .utils import combine
from scipy.spatial.transform import Rotation as R



class ServoControl:
    def __init__(self, robot):
        self.frequency = 1/500

        self.threshold = 20
        self.contactMean = 1 #Newton

        self.robot = robot

    def convertForceInBase2TCP(self,ftInBase):
        tcpPose = self.robot.getActualTCPPose()
        rot = R.from_rotvec(tcpPose[3:6])
        rMatrix = scipy.linalg.inv(rot.as_matrix())
        return rMatrix @ ftInBase[0:3]
        
    def saveForceVector(self, vector, pose):
        with open("forceVector.txt", 'w+') as file:
            string = ""
            for value in pose:
                string += str(value) + " "
            for value in vector:
                string += str(value) + " "
            string = string[0:-1]
            file.write(string)

    def run(self, positionalTrajectory, scaling = 1, velocity = 0.5, acceleration = 0.5, lookAheadTime = 0.1, gain = 300, runCusum = True, nrOfPointsToBacktrack = 400):
        

        pose = positionalTrajectory[0]
        #print(pose, type(pose))
        self.robot.moveL(pose)
        pointsTouched = 0
        backtrackedTraj = []
        time.sleep(0.3)
        magnitudeList = []
        cusumValues = [0]
        self.robot.zeroFtSensor()
        resetTime = time.time()
        for i, point in enumerate(positionalTrajectory):
            startTime = time.time()

            pose = point
            self.robot.servoL(pose, velocity, acceleration, self.frequency/2, lookAheadTime, gain)
            pointsTouched += 1


            if runCusum:
                ftInBase = self.robot.getActualTCPForce()
                ftInTCP = self.convertForceInBase2TCP(ftInBase)
                magnitudeList.append(np.linalg.norm(ftInTCP))

                mean = np.asarray(magnitudeList).mean()
                cusumValues.append(np.maximum(0, cusumValues[-1] + (mean - self.contactMean)))
                #print(cusumValues[-1])

                if cusumValues[-1] > self.threshold:
                    if i > nrOfPointsToBacktrack:
                        backtrackedTraj = positionalTrajectory[i-nrOfPointsToBacktrack:i]
                    else:
                        backtrackedTraj = positionalTrajectory[0:i]
                    break

            if time.time() - resetTime > 0.1:
                self.robot.zeroFtSensor()
                resetTime = time.time()
            
            diff = time.time() - startTime
            if(diff < self.frequency):
                time.sleep(self.frequency - diff)
        
        self.robot.servoStop()
        if cusumValues[-1] > self.threshold:
            print("Contact!")
            self.saveForceVector(ftInTCP, self.robot.getActualTCPPose())
        else:
            print("No contact!")
        time.sleep(0.1)
        return backtrackedTraj
    
    def getForceVector(self):
        with open("forceVector.txt", 'r') as file:
            vector = file.readline().split(" ")
        vector = [float(i) for i in vector]
        '''
        u = np.sin(np.pi * vector[0]) * np.cos(np.pi * vector[1]) * np.cos(np.pi * vector[2])
        v = -np.cos(np.pi * vector[0]) * np.sin(np.pi * vector[1]) * np.cos(np.pi * vector[2])
        w = np.sqrt(2/3) * np.cos(np.pi * vector[0]) * np.cos(np.pi * vector[1]) * np.sin(np.pi * vector[2])
        vector.append(u)
        vector.append(v)
        vector.append(w)
        '''
        return vector

    def generatePointsToVisit(self, vector, nrOfPoints = 4, size = 0.05):
        nVector = vector[6:9]
        point = vector[0:3]
        Q = scipy.linalg.null_space(np.asmatrix(nVector))
        randomPoints = np.random.rand(2,nrOfPoints) - 0.5
        points = np.asarray(point).reshape((3,1)) + np.matmul(Q,randomPoints*size)
        return points
    
    def addPointsTogether(self, p1, p2):
        for p in p2.T:
            p1.append(p)
        return p1


    def recursiveSearch(self, startPoint, contactPoints, forceVectors, allPoints, depth, nrOfPointsToBacktrack = 2000):
        if depth > 3:
            return contactPoints, forceVectors, allPoints
        forceVector = self.getForceVector()
        forceVectors.append(forceVector)
        points = self.generatePointsToVisit(forceVector)
        allPoints = self.addPointsTogether(allPoints, points)
        for point in points.T:
            traj = np.linspace(startPoint, point, nrOfPointsToBacktrack)
            traj = combine(traj)
            backtrackedTrajectory = self.run(traj, scaling=1)
            if backtrackedTrajectory:
                contactPoints.append(backtrackedTrajectory[-1][0:3])
                self.recursiveSearch(startPoint, contactPoints, forceVectors, allPoints, depth+1)
            time.sleep(0.1)
        return contactPoints, forceVectors, allPoints


    def searchForObstacle(self, backtrackedTraj):
        print("Searching for obstacle!")
        backtrackedTraj = list(reversed(backtrackedTraj))
        nrOfPointsToBacktrack = 400
        self.run(backtrackedTraj, runCusum=False, nrOfPointsToBacktrack=nrOfPointsToBacktrack)
        time.sleep(1)
        contactPoints, forceVectors, allPoints = self.recursiveSearch(backtrackedTraj[-1][0:3], [backtrackedTraj[0][0:3]], [self.getForceVector()], [],0)
        contactPoints = np.asmatrix(contactPoints)
        forceVectors = np.asmatrix(forceVectors)
        allPoints = np.asmatrix(allPoints)
        #print(contactPoints, contactPoints.shape, "\n", forceVectors, forceVectors.shape, allPoints.shape)
        return contactPoints.T, forceVectors.T, allPoints.T
        
