import time
import threading
from .FTSensor.FTSensor import FTSensor
import numpy as np
import scipy
from .utils import combine
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt



class ServoControl:
    def __init__(self, robot):
        self.frequency = 1/500

        self.threshold = 70
        self.contactMean = 2 * 2 #Newton
        self.contactSTD = 1

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

    def run(self, positionalTrajectory, scaling = 1, velocity = 0.5, acceleration = 0.5, lookAheadTime = 0.1, gain = 600, runCusum = True, nrOfPointsToBacktrack = 400):
        

        pose = positionalTrajectory[0]
        #print(pose, type(pose))
        self.robot.moveL(pose)
        pointsTouched = 0
        backtrackedTraj = []
        time.sleep(0.3)
        magnitudeList = []
        accelerometer = []
        ftInTCP_ = []
        tcpPoses = []
        accelerometerMagnitude = []
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
                ftInTCP_.append(ftInTCP)
                tcpPoses.append(self.robot.getActualTCPPose())
                magnitudeList.append(np.linalg.norm(ftInTCP))
                accelerometer.append(np.asarray(self.robot.getActualToolAccelerometer()) + np.asarray([0.08558124291377506, 0.1021011405616492, 9.171799696940514]))
                accelerometerMagnitude.append(np.linalg.norm(accelerometer[-1]))

                mean = np.asarray(magnitudeList).mean()
                sk = ((magnitudeList[-1] - mean)**2-(magnitudeList[-1] - self.contactMean)**2)/(2*self.contactSTD**2)
                cusumValues.append(np.maximum(0, cusumValues[-1] + sk))
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
            #self.plotRun(magnitudeList, np.asarray(accelerometer).T,accelerometerMagnitude, cusumValues[1:], np.asarray(ftInTCP_).T, np.asarray(tcpPoses).T)
        else:
            print("No contact!")
        time.sleep(0.1)
        return backtrackedTraj
    
    def plotRun(self,ftMagnitude, accelerometer,accelerometerMagnitude,cusumValues, ftInTCP_, tcpPoses):
        xAxis = range(0,len(ftMagnitude))

        plot1 = plt.figure(1)
        axes = plt.gca()
        plt.plot(xAxis,ftMagnitude, color='r', label="Force magnitude")
        #plt.plot(xAxis,accelerometerMagnitude, color='g', label='Accelerometer Magnitude')
        plt.plot(xAxis, cusumValues, color='black', label="CUSUM Values")
        axes.set_ylim([-1,8])
        plt.legend()

        fig1, axs = plt.subplots(3, 3, sharex=True)
        axs[0,0].plot(xAxis, accelerometer[0,:], label="Acceleration")
        axs[0,0].set_xlabel("# Measurement")
        axs[0,0].title.set_text("x-axis")
        axs[0,0].set_ylabel("m/s^2")

        axs[1,0].plot(xAxis, accelerometer[1,:], label="Acceleration")
        axs[1,0].set_xlabel("# Measurement")
        axs[1,0].title.set_text("y-axis")
        axs[1,0].set_ylabel("m/s^2")

        axs[2,0].plot(xAxis, accelerometer[2,:], label="Acceleration")
        axs[2,0].set_xlabel("# Measurement")
        axs[2,0].title.set_text("z-axis")
        axs[2,0].set_ylabel("m/s^2")
        axs[2,0].legend()

        axs[0,1].plot(xAxis, ftInTCP_[0,:], color='orange', label="Force")
        axs[0,1].set_xlabel("# Measurement")
        axs[0,1].title.set_text("x-axis")
        axs[0,1].set_ylabel("F [N]")

        axs[1,1].plot(xAxis, ftInTCP_[1,:], color='orange', label="Force")
        axs[1,1].set_xlabel("# Measurement")
        axs[1,1].title.set_text("y-axis")
        axs[1,1].set_ylabel("F [N]")

        axs[2,1].plot(xAxis, ftInTCP_[2,:], color='orange', label="Force")
        axs[2,1].set_xlabel("# Measurement")
        axs[2,1].title.set_text("z-axis")
        axs[2,1].set_ylabel("F [N]")
        axs[2,1].legend()

        axs[0,2].plot(xAxis, tcpPoses[0,:], color='black', label="TCP Position")
        axs[0,2].set_xlabel("# Measurement")
        axs[0,2].set_ylabel("x-coordinate")
        axs[0,2].title.set_text("x-axis")

        axs[1,2].plot(xAxis, tcpPoses[1,:], color='black', label="TCP Position")
        axs[1,2].set_xlabel("# Measurement")
        axs[1,2].set_ylabel("y-coordinate")
        axs[1,2].title.set_text("y-axis")

        axs[2,2].plot(xAxis, tcpPoses[2,:], color='black', label="TCP Position")
        axs[2,2].set_xlabel("# Measurement")
        axs[2,2].set_ylabel("z-coordinate")
        axs[2,2].title.set_text("z-axis")
        axs[2,2].legend()


        plt.show()

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

    def generatePointsToVisit(self, vector, nrOfPoints = 3, size = 0.1):
        nVector = vector[6:9]
        point = vector[0:3]
        Q = scipy.linalg.null_space(np.asmatrix(nVector))
        randomPoints = np.random.rand(2,nrOfPoints) - 0.5
        offsetPointsInVectorDirection = (np.asarray(nVector).reshape(3,1)/np.linalg.norm(nVector))*0.05
        points = np.asarray(point).reshape((3,1)) + np.matmul(Q,randomPoints*size) - offsetPointsInVectorDirection
        return points
    
    def addPointsTogether(self, p1, p2):
        for p in p2.T:
            p1.append(p)
        return p1


    def recursiveSearch(self, startPoint, contactPoints, forceVectors, allPoints, depth, nrOfPointsToBacktrack = 2000):
        forceVector = self.getForceVector()
        forceVectors.append(forceVector)
        if depth >= 2:
            return contactPoints, forceVectors, allPoints
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
        
