import time
import threading
from .FTSensor.FTSensor import FTSensor
import numpy as np
import scipy
from .utils import combine



class ServoControl:
    def __init__(self, robot):
        self.frequency = 1/500

        self.robot = robot

    def run(self, event, positionalTrajectory, scaling = 1, velocity = 0.5, acceleration = 0.5, lookAheadTime = 0.1, gain = 300, runCusum = True, nrOfPointsToBacktrack = 400):
        #print("Running DMP")

        if runCusum:
            ft = FTSensor(self.robot)
            forceThread = threading.Thread(target=ft.cusum, args=(event, 20,))
            forceThread.start()
            event.set()

        

        pose = positionalTrajectory[0]
        #print(pose, type(pose))
        self.robot.moveL(pose)
        pointsTouched = 0
        backtrackedTraj = []
        time.sleep(0.1)
        for i, point in enumerate(positionalTrajectory):
            startTime = time.time()

            pose = point
            self.robot.servoL(pose, velocity, acceleration, self.frequency/2, lookAheadTime, gain)
            pointsTouched += 1

            if event.is_set() and runCusum:
                if i > nrOfPointsToBacktrack:
                    backtrackedTraj = positionalTrajectory[i-nrOfPointsToBacktrack:i]
                else:
                    backtrackedTraj = positionalTrajectory[0:i]
                break

            
            
            diff = time.time() - startTime
            if(diff < self.frequency):
                time.sleep(self.frequency - diff)
        
        self.robot.servoStop()
        event.set()
        print("DONE")
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

    def generatePointsToVisit(self, vector, nrOfPoints = 4, size = 0.1):
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


    def recursiveSearch(self, event, startPoint, contactPoints, forceVectors, allPoints, depth, nrOfPointsToBacktrack = 400):
        if depth > 3:
            return contactPoints, forceVectors, allPoints
        forceVector = self.getForceVector()
        forceVectors.append(forceVector)
        points = self.generatePointsToVisit(forceVector)
        allPoints = self.addPointsTogether(allPoints, points)
        for point in points.T:
            traj = np.linspace(startPoint, point, nrOfPointsToBacktrack)
            traj = combine(traj)
            event.clear()
            backtrackedTrajectory = self.run(event, traj, scaling=3)
            if backtrackedTrajectory:
                contactPoints.append(backtrackedTrajectory[-1][0:3])
                self.recursiveSearch(event, startPoint, contactPoints, forceVectors, allPoints, depth+1)
            time.sleep(0.1)
        return contactPoints, forceVectors, allPoints


    def searchForObstacle(self, event, backtrackedTraj):
        print("Searching for obstacle!")
        backtrackedTraj = list(reversed(backtrackedTraj))
        nrOfPointsToBacktrack = 400
        self.run(event, backtrackedTraj, runCusum=False, nrOfPointsToBacktrack=nrOfPointsToBacktrack)
        event.clear()
        time.sleep(1)
        contactPoints, forceVectors, allPoints = self.recursiveSearch(event, backtrackedTraj[-1][0:3], [backtrackedTraj[0][0:3]], [self.getForceVector()], [],0)
        contactPoints = np.asmatrix(contactPoints)
        forceVectors = np.asmatrix(forceVectors)
        allPoints = np.asmatrix(allPoints)
        #print(contactPoints, contactPoints.shape, "\n", forceVectors, forceVectors.shape, allPoints.shape)
        return contactPoints.T, forceVectors.T, allPoints.T
        
