import time
import numpy as np
import scipy
from utils import combine, NOISE_FILENAME, FORCEVECTOR_FILENAME
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt



class ServoControl:
    def __init__(self, robot):
        self.frequency = 1/500

        self.threshold = 70
        self.contactMean = 3 * 2 #Newton
        self.contactSTD = 1

        self.robot = robot

    def convertForceInBase2TCP(self,ftInBase):
        tcpPose = self.robot.getActualTCPPose()
        rot = R.from_rotvec(tcpPose[3:6])
        rMatrix = scipy.linalg.inv(rot.as_matrix())
        return rMatrix @ ftInBase[0:3]
        
    def saveForceVector(self, vector, pose):
        with open(FORCEVECTOR_FILENAME, 'w+') as file:
            string = ""
            for value in pose:
                string += str(value) + " "
            for value in vector:
                string += str(value) + " "
            string = string[0:-1]
            file.write(string)

    def loadNoiseValues(self):
            with open(NOISE_FILENAME, 'r') as file:
                values = file.readline().split(" ")
            #noiseMean =  float(values[0])
            noiseSTD = float(values[1])
            #acceleration = [float(acc) for acc in values[2:-1]]
            self.contactSTD = noiseSTD
            

    def run(self, positionalTrajectory, velocity = 0.5, acceleration = 0.5, lookAheadTime = 0.1, gain = 600, runCusum = True, nrOfPointsToBacktrack = 400):
        self.loadNoiseValues()
        
        pose = positionalTrajectory[0]
        self.robot.moveL(pose)

        backtrackedTraj = []
        magnitudeList = []
        ftsInTCP = []
        tcpPoss = []
        cusumValues = [0]

        self.robot.zeroFtSensor()
        resetTime = time.time()
        for i, point in enumerate(positionalTrajectory):
            startTime = time.time()

            self.robot.servoL(point, velocity, acceleration, self.frequency/2, lookAheadTime, gain)


            if runCusum:
                ftInTCP = self.convertForceInBase2TCP(self.robot.getActualTCPForce())
                ftsInTCP.append(ftInTCP)

                tcpPoss.append(self.robot.getActualTCPPose()[0:3])

                magnitudeList.append(np.linalg.norm(ftInTCP))

                mean = np.asarray(magnitudeList).mean()
                sk = ((magnitudeList[-1] - mean)**2-(magnitudeList[-1] - self.contactMean)**2)/(2*self.contactSTD**2)
                cusumValues.append(np.maximum(0, cusumValues[-1] + sk))

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
            #self.plotRun(magnitudeList, cusumValues[1:], np.asarray(ftsInTCP).T, np.asarray(tcpPoss).T)
        elif runCusum:
            print("No contact!")
        time.sleep(0.01)
        return backtrackedTraj
    
    def plotRun(self,ftMagnitude, cusumValues, ftsInTCP, tcpPoss):
        xAxis = range(0,len(ftMagnitude))

        plot1 = plt.figure(1)
        axes = plt.gca()
        plt.plot(xAxis,ftMagnitude, color='r', label="Force magnitude [N]")
        plt.plot(xAxis, cusumValues, color='black', label="CUSUM Values")
        plt.xlabel("# Measurement")
        axes.set_ylim([-1,8])
        plt.legend()

        fig1, axs = plt.subplots(3, 2, sharex=True)

        axs[0,0].plot(xAxis, ftsInTCP[0,:], color='orange', label="Force")
        axs[0,0].set_xlabel("# Measurement")
        axs[0,0].title.set_text("x-axis")
        axs[0,0].set_ylabel("F [N]")

        axs[1,0].plot(xAxis, ftsInTCP[1,:], color='orange', label="Force")
        axs[1,0].set_xlabel("# Measurement")
        axs[1,0].title.set_text("y-axis")
        axs[1,0].set_ylabel("F [N]")

        axs[2,0].plot(xAxis, ftsInTCP[2,:], color='orange', label="Force")
        axs[2,0].set_xlabel("# Measurement")
        axs[2,0].title.set_text("z-axis")
        axs[2,0].set_ylabel("F [N]")
        axs[2,0].legend()

        axs[0,1].plot(xAxis, tcpPoss[0,:], color='black', label="TCP Position")
        axs[0,1].set_xlabel("# Measurement")
        axs[0,1].set_ylabel("x-coordinate")
        axs[0,1].title.set_text("x-axis")

        axs[1,1].plot(xAxis, tcpPoss[1,:], color='black', label="TCP Position")
        axs[1,1].set_xlabel("# Measurement")
        axs[1,1].set_ylabel("y-coordinate")
        axs[1,1].title.set_text("y-axis")

        axs[2,1].plot(xAxis, tcpPoss[2,:], color='black', label="TCP Position")
        axs[2,1].set_xlabel("# Measurement")
        axs[2,1].set_ylabel("z-coordinate")
        axs[2,1].title.set_text("z-axis")
        axs[2,1].legend()


        plt.show()

    def getForceVector(self):
        with open(FORCEVECTOR_FILENAME, 'r') as file:
            vector = file.readline().split(" ")
        vector = [float(i) for i in vector]
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
            backtrackedTrajectory = self.run(traj)
            if backtrackedTrajectory:
                contactPoints.append(backtrackedTrajectory[-1][0:3])
                self.recursiveSearch(startPoint, contactPoints, forceVectors, allPoints, depth+1)
            time.sleep(0.05)
        return contactPoints, forceVectors, allPoints


    def searchForObstacle(self, backtrackedTraj, nrOfPointsToBacktrack_ = 400):
        print("Searching for obstacle!")
        backtrackedTraj = list(reversed(backtrackedTraj))
        nrOfPointsToBacktrack = nrOfPointsToBacktrack_
        self.run(backtrackedTraj, runCusum=False, nrOfPointsToBacktrack=nrOfPointsToBacktrack)
        time.sleep(0.3)
        contactPoints, forceVectors, allPoints = self.recursiveSearch(backtrackedTraj[-1][0:3], [backtrackedTraj[0][0:3]], [self.getForceVector()], [],0)
        contactPoints = np.asmatrix(contactPoints)
        forceVectors = np.asmatrix(forceVectors)
        allPoints = np.asmatrix(allPoints)
        return contactPoints.T, forceVectors.T, allPoints.T
        
