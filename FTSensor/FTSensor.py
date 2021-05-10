import time
from scipy.spatial.transform import Rotation as R
import scipy
import numpy as np


class FTSensor:
    def __init__(self, robot):
        self.noiseMagnitudeOfForce = []
        self.frequency = 1/500
        self.robot = robot
        self.cusumHighThreshold = 20
        self.noiseMean = 0
        self.noiseSTD = 1
        self.contactMean = 1.5 # 5 Newton
        self.contactSTD = 1

    def convertForceInBase2TCP(self,ftInBase):
        tcpPose = self.robot.getActualTCPPose()
        rot = R.from_rotvec(tcpPose[3:6])
        rMatrix = scipy.linalg.inv(rot.as_matrix())
        return rMatrix @ ftInBase[0:3]

    def noiseFT(self, duration):
        print("Recording noise F/T sensor!")
        durTime = time.time()
        while time.time() - durTime < duration:
            startTime = time.time()

            ftInBase = self.robot.getActualTCPForce()
            ftInTCP = self.convertForceInBase2TCP(ftInBase)
            
            magnitudeOfForce = np.linalg.norm(ftInTCP)
            self.noiseMagnitudeOfForce.append(magnitudeOfForce)


            diff = time.time() - startTime
            if(diff < self.frequency):
                time.sleep(self.frequency - diff)
        
        self.noiseMean = np.asarray(self.noiseMagnitudeOfForce).mean()
        self.noiseSTD = np.asarray(self.noiseMagnitudeOfForce).std()
        print(self.noiseMean, self.noiseSTD)
        self.contactSTD = self.noiseSTD

    def cusum(self, nrOfObsPerComparision):
        cusumValue = 0
        magnitudeList = []
        cusumValues = [0]
        while self.cusumHighThreshold > cusumValue:
            startTime = time.time()

            ftInBase = self.robot.getActualTCPForce()
            ftInTCP = self.convertForceInBase2TCP(ftInBase)

            magnitudeList.append(np.linalg.norm(ftInTCP))
            if len(magnitudeList) > nrOfObsPerComparision:
                pass
                #magnitudeList.pop()
            
            
            mean = np.asarray(magnitudeList).mean()
            #std = np.asarray(magnitudeList).std()

            normalizedMean = (mean - self.contactMean)/self.contactSTD
            #print(magnitudeList[-1], cusumValues[-1], mean, normalizedMean)
            cusumValue = cusumValues[-1]+normalizedMean
            cusumValue = np.maximum(0,cusumValue)
            cusumValues.append(cusumValue)
            #print(cusumValue)




            diff = time.time() - startTime
            if(diff < self.frequency):
                time.sleep(self.frequency - diff)
        print("Contact!")