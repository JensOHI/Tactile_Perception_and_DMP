import time
from scipy.spatial.transform import Rotation as R
import scipy
import numpy as np
import matplotlib.pyplot as plt


class FTSensor:
    def __init__(self, robot):
        self.noiseMagnitudeOfForce = []
        self.frequency = 1/500
        self.robot = robot
        self.cusumHighThreshold = 40
        self.noiseMean = 0
        self.noiseSTD = 1
        self.contactMean = 0.5 #Newton
        self.contactSTD = 1
        self.acceleration = []
        self.payloadMass = 3
        self.filename = "noiseValues.txt"

    def convertForceInBase2TCP(self,ftInBase):
        tcpPose = self.robot.getActualTCPPose()
        rot = R.from_rotvec(tcpPose[3:6])
        rMatrix = scipy.linalg.inv(rot.as_matrix())
        return rMatrix @ ftInBase[0:3]

    def saveNoiseValues(self):
        print("Saving",self.filename)
        with open(self.filename, 'w') as file:
            string = str(self.noiseMean) + " " + str(self.noiseSTD) + " " + str(self.acceleration)[1:-1]
            file.write(string)

    def noiseFT(self, duration):
        print("Recording noise F/T sensor!")
        durTime = time.time()
        self.robot.zeroFtSensor()
        accelerations = []
        while time.time() - durTime < duration:
            #print("\r"+"Time left:"+str(time.time()-durTime),end=" "*20)
            startTime = time.time()

            ftInBase = self.robot.getActualTCPForce()
            ftInTCP = self.convertForceInBase2TCP(ftInBase)
            
            magnitudeOfForce = np.linalg.norm(ftInTCP)
            self.noiseMagnitudeOfForce.append(magnitudeOfForce)
            accelerations.append(self.robot.getActualToolAccelerometer())

            diff = time.time() - startTime
            if(diff < self.frequency):
                time.sleep(self.frequency - diff)
        
        self.noiseMean = np.asarray(self.noiseMagnitudeOfForce).mean()
        self.noiseSTD = np.asarray(self.noiseMagnitudeOfForce).std()
        self.contactSTD = self.noiseSTD
        self.acceleration = np.mean(np.asarray(accelerations),axis=0)
        self.saveNoiseValues()
        print("Noise mean:", self.noiseMean, "Noise std:", self.noiseSTD,"Acceleration:",self.acceleration)

    def saveForceVector(self, vector, pose):
        with open("forceVector.txt", 'w+') as file:
            string = ""
            for value in pose:
                string += str(value) + " "
            for value in vector:
                string += str(value) + " "
            string = string[0:-1]
            file.write(string)
    
    def loadNoiseValues(self):
        print("Loading",self.filename)
        with open(self.filename, 'r') as file:
            values = file.readline.split(" ")
        self.noiseMean =  float(values[0])
        self.noiseSTD = float(values[1])
        self.acceleration = [float(acc) for acc in values[2:-1]]
        self.contactSTD = self.noiseSTD
        print(self.noiseMean,self.noiseSTD,self.acceleration)

    def cusum(self, event, nrOfObsPerComparision):
        self.loadNoiseValues()
        cusumValue = 0
        magnitudeList = []
        cusumValues = [0]
        print("Waiting for servoControl to start!")
        event.wait()
        event.clear()
        
        
        accelerations = []
        inertiaCompensations = []
        tcpForces = []
        self.robot.zeroFtSensor()
        resetTime = time.time()
        cusumCancelled = False
        print("Starting CUSUM!")
        while self.cusumHighThreshold > cusumValue:
            startTime = time.time()

            if event.is_set():
                cusumCancelled = True
                event.clear()
                break

           
            ftInBase = self.robot.getActualTCPForce()
            #print(self.robot.getActualToolAccelerometer(), self.acceleration, self.payloadMass)
            accelerations.append(self.robot.getActualToolAccelerometer())
            inertiaCompensation = (abs(np.asarray(accelerations[-1])) - abs(np.asarray(self.acceleration))) * self.payloadMass
            inertiaCompensations.append(inertiaCompensation)
            ftInTCP = self.convertForceInBase2TCP(ftInBase)# - inertiaCompensation
            tcpForces.append(ftInTCP)
            
            magnitudeList.append(np.linalg.norm(ftInTCP))
            #print(inertiaCompensation, ftInTCP, np.linalg.norm(inertiaCompensation), magnitudeList[-1])
            
            
            mean = np.asarray(magnitudeList).mean()
            #std = np.asarray(magnitudeList).std()

            normalizedMean = (mean - self.contactMean)#/self.contactSTD
            #print(magnitudeList[-1], cusumValues[-1], mean, normalizedMean)
            cusumValue = cusumValues[-1]+normalizedMean
            cusumValue = np.maximum(0,cusumValue)
            cusumValues.append(cusumValue)
            #print(cusumValue)

            if time.time() - resetTime > 0.1:
                self.robot.zeroFtSensor()
                resetTime = time.time()

            #self.robot.zeroFtSensor()
            diff = time.time() - startTime
            if(diff < self.frequency):
                time.sleep(self.frequency - diff)
        if not cusumCancelled:
            print("Contact!")
            event.set()
            self.saveForceVector(ftInTCP, self.robot.getActualTCPPose())
            #self.plot(accelerations, inertiaCompensations, tcpForces)
        else:
            print("No contact detected. CUSUM STOPPED!")

    def plot(self, accelerations, inertiaCompensations, tcpForces):
        normAcc = np.asarray([np.linalg.norm(acc) for acc in accelerations])
        normIC = np.asarray([np.linalg.norm(ic) for ic in inertiaCompensations])
        normTCPForces = np.asarray([np.linalg.norm(tcpForce) for tcpForce in tcpForces])
        xAxis = range(0,len(normAcc))
        plt.plot(xAxis, normAcc, label="Acceleration magnitude")
        plt.plot(xAxis, normIC, label="Inertia Compensation magnitude")
        plt.plot(xAxis, normTCPForces, label="TCP forces magnitude")
        #plt.plot(xAxis, normTCPForces-normIC, label="TCP Force - Acceleration")
        plt.legend()
        plt.ylim([-1,12])
        


        fig1, axs = plt.subplots(3, 1, sharex=True)
        axs[0].plot(xAxis, np.asarray(accelerations)[:,0], label="Acceleration")
        axs[0].plot(xAxis, np.asarray(inertiaCompensations)[:,0], label="Inertia Compensation [mass = "+str(self.payloadMass)+"]")
        axs[0].plot(xAxis, np.asarray(tcpForces)[:,0], label="TCP Force")
        axs[0].set_xlabel("# Measurement")
        axs[0].set_ylabel("F [N]")
        axs[0].title.set_text("x-axis")

        axs[1].plot(xAxis, np.asarray(accelerations)[:,1], label="Acceleration")
        axs[1].plot(xAxis, np.asarray(inertiaCompensations)[:,1], label="Inertia Compensation [mass = "+str(self.payloadMass)+"]")
        axs[1].plot(xAxis, np.asarray(tcpForces)[:,1], label="TCP Force")
        axs[1].set_xlabel("# Measurement")
        axs[1].set_ylabel("F [N]")
        axs[1].title.set_text("y-axis")

        axs[2].plot(xAxis, np.asarray(accelerations)[:,2], label="Acceleration")
        axs[2].plot(xAxis, np.asarray(inertiaCompensations)[:,2], label="Inertia Compensation [mass = "+str(self.payloadMass)+"]")
        axs[2].plot(xAxis, np.asarray(tcpForces)[:,2], label="TCP Force")
        axs[2].set_xlabel("# Measurement")
        axs[2].set_ylabel("F [N]")
        axs[2].title.set_text("z-axis")
        axs[2].legend()

        plt.show()

