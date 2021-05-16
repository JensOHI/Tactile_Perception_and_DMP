import time
from scipy.spatial.transform import Rotation as R
import scipy
import numpy as np
import matplotlib.pyplot as plt
from utils import NOISE_FILENAME


class FTSensor:
    def __init__(self, robot):
        self.frequency = 1/500
        self.robot = robot
        self.noiseMean = 0
        self.noiseSTD = 1
        self.acceleration = []
        self.filename = NOISE_FILENAME

    def convertForceInBase2TCP(self,ftInBase):
        tcpPose = self.robot.getActualTCPPose()
        rot = R.from_rotvec(tcpPose[3:6])
        rMatrix = scipy.linalg.inv(rot.as_matrix())
        return rMatrix @ ftInBase[0:3]

    def saveNoiseValues(self):
        print("Saving",self.filename)
        with open(self.filename, 'w+') as file:
            string = str(self.noiseMean) + " " + str(self.noiseSTD) + " " + str(self.acceleration)[1:-1]
            file.write(string)

    def noiseFT(self, duration):
        print("Recording noise F/T sensor!")
        
        durTime = time.time()
        self.robot.zeroFtSensor()
        accelerations = []
        noiseMagnitudeOfForce = []
        while time.time() - durTime < duration:
            startTime = time.time()

            ftInBase = self.robot.getActualTCPForce()
            ftInTCP = self.convertForceInBase2TCP(ftInBase)

            noiseMagnitudeOfForce.append(np.linalg.norm(ftInTCP))

            accelerations.append(self.robot.getActualToolAccelerometer())

            diff = time.time() - startTime
            if(diff < self.frequency):
                time.sleep(self.frequency - diff)
        
        self.noiseMean = np.asarray(noiseMagnitudeOfForce).mean()
        self.noiseSTD = np.asarray(noiseMagnitudeOfForce).std()
        self.acceleration = np.mean(np.asarray(accelerations),axis=0)
        self.saveNoiseValues()
        print("Noise mean:", self.noiseMean, "Noise std:", self.noiseSTD,"Acceleration:",self.acceleration)


   

