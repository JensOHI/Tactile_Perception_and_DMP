from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
from rtde_io import RTDEIOInterface
import numpy as np
import time
import matplotlib.pyplot as plt
from UR5_code.Robot import Robot
from scipy.spatial.transform import Rotation as R

def convertAccInTCP2Base(robot,accInTCP):
        tcpPose = robot.getActualTCPPose()
        rot = R.from_rotvec(tcpPose[3:6])
        rMatrix = rot.as_matrix()
        return rMatrix @ accInTCP

def plot(acceleration, ftSensors, inertiaCompensations, ftCompensations, cusumValues, pos, ftMagnitude, filteredFTMagnitude):
    print(acceleration.shape, ftSensors.shape, inertiaCompensations.shape, ftCompensations.shape, len(cusumValues), pos.shape, len(ftMagnitude), len(filteredFTMagnitude))
    xAxis = range(0,np.maximum(acceleration.shape[0],acceleration.shape[1]))

    plot1 = plt.figure(1)
    plt.plot(xAxis, cusumValues, label="CUSUM Values")
    plt.plot(xAxis,ftMagnitude, color='r', label="Force magnitude")
    #plt.plot(xAxis, filteredFTMagnitude, color='g', label="Filtered magnitude")
    plt.legend()

    fig1, axs = plt.subplots(3, 5, sharex=True)

    # Acceleration
    axs[0,0].plot(xAxis, acceleration[0,:], label="Acceleration")
    axs[0,0].set_xlabel("# Measurement")
    axs[0,0].title.set_text("x-axis")
    axs[0,0].set_ylabel("m/s^2")

    axs[1,0].plot(xAxis, acceleration[1,:], label="Acceleration")
    axs[1,0].set_xlabel("# Measurement")
    axs[1,0].title.set_text("y-axis")
    axs[1,0].set_ylabel("m/s^2")

    axs[2,0].plot(xAxis, acceleration[2,:], label="Acceleration")
    axs[2,0].set_xlabel("# Measurement")
    axs[2,0].title.set_text("z-axis")
    axs[2,0].set_ylabel("m/s^2")
    axs[2,0].legend()

    # Inertia Compensation
    axs[0,1].plot(xAxis, inertiaCompensations[0,:], color="g", label="inertiaCompensations")
    axs[0,1].set_xlabel("# Measurement")
    axs[0,1].title.set_text("x-axis")
    axs[0,1].set_ylabel("m/s^2")

    axs[1,1].plot(xAxis, inertiaCompensations[1,:], color="g", label="inertiaCompensations")
    axs[1,1].set_xlabel("# Measurement")
    axs[1,1].title.set_text("y-axis")
    axs[1,1].set_ylabel("m/s^2")

    axs[2,1].plot(xAxis, inertiaCompensations[2,:], color="g", label="inertiaCompensations")
    axs[2,1].set_xlabel("# Measurement")
    axs[2,1].title.set_text("z-axis")
    axs[2,1].set_ylabel("m/s^2")
    axs[2,1].legend()

    #Force
    axs[0,2].plot(xAxis, ftSensors[0,:], color='orange', label="Force")
    axs[0,2].set_xlabel("# Measurement")
    axs[0,2].title.set_text("x-axis")
    axs[0,2].set_ylabel("F [N]")

    axs[1,2].plot(xAxis, ftSensors[1,:], color='orange', label="Force")
    axs[1,2].set_xlabel("# Measurement")
    axs[1,2].title.set_text("y-axis")
    axs[1,2].set_ylabel("F [N]")

    axs[2,2].plot(xAxis, ftSensors[2,:], color='orange', label="Force")
    axs[2,2].set_xlabel("# Measurement")
    axs[2,2].title.set_text("z-axis")
    axs[2,2].set_ylabel("F [N]")
    axs[2,2].legend()

    #Force Compensation
    axs[0,3].plot(xAxis, ftCompensations[0,:], color='r', label="Force Compensation")
    axs[0,3].set_xlabel("# Measurement")
    axs[0,3].set_ylabel("F [N]")
    axs[0,3].title.set_text("x-axis")

    axs[1,3].plot(xAxis, ftCompensations[1,:], color='r', label="Force Compensation")
    axs[1,3].set_xlabel("# Measurement")
    axs[1,3].set_ylabel("F [N]")
    axs[1,3].title.set_text("y-axis")

    axs[2,3].plot(xAxis, ftCompensations[2,:], color='r', label="Force Compensation")
    axs[2,3].set_xlabel("# Measurement")
    axs[2,3].title.set_text("z-axis")
    axs[2,3].set_ylabel("F [N]")
    axs[2,3].legend()

    #Force Compensation
    axs[0,4].plot(xAxis, pos[0,:], color='black', label="TCP Position")
    axs[0,4].set_xlabel("# Measurement")
    axs[0,4].set_ylabel("x-coordinate")
    axs[0,4].title.set_text("x-axis")

    axs[1,4].plot(xAxis, pos[1,:], color='black', label="TCP Position")
    axs[1,4].set_xlabel("# Measurement")
    axs[1,4].set_ylabel("y-coordinate")
    axs[1,4].title.set_text("y-axis")

    axs[2,4].plot(xAxis, pos[2,:], color='black', label="TCP Position")
    axs[2,4].set_xlabel("# Measurement")
    axs[2,4].set_ylabel("z-coordinate")
    axs[2,4].title.set_text("z-axis")
    axs[2,4].legend()

    plt.show()



def main():
    ip = "192.168.1.111"
    robot = Robot(ip)


    homeL = [-0.4333274207352126, 0.03674068343275796, 0.3958794499167509, 0.53821750029029, 3.0950166376040498, 4.7329968045906084e-05]
    goalL = [-0.7317612813142724, 0.03674068343275796, 0.3958794499167509, 0.53821750029029, 3.0950166376040498, 4.7329968045906084e-05]

    robot.moveL(homeL)

    nrOfPoints = int(500)
    traj = np.linspace(homeL, goalL, nrOfPoints)

    hz = 500
    frequency = 1.0/hz
    payload = 0.6

    accelerations = []
    inertiaCompensations = []
    ftSensors = [[0,0,0]]
    ftMagnitude = [0]
    filteredFTMagnitude = []
    ftCompensations = []
    diffFTs = []
    pos = []

    noiseAcc = []
    noiseFT = []
    t1 = time.time()
    while time.time() - t1 < 1.5:
        noiseFT.append(np.linalg.norm(robot.getActualTCPPose()[0:3]))
        noiseAcc.append(convertAccInTCP2Base(robot, robot.getActualToolAccelerometer()))
    avgNoiseAcc = np.mean(np.asarray(noiseAcc), axis=0)
    noiseMean = np.mean(np.asarray(noiseFT))
    noiseSTD = 1#np.std(np.asarray(noiseFT))
    print(noiseMean, noiseSTD)
    cusumValues = [0]

    contactMean = 5
    threshold = 70

    i = 0
    count = 0
    robot.zeroFtSensor()
    resetTime = time.time()
    while count < nrOfPoints*4:
        startTime = time.time()

        robot.servoL(traj[i], 0.5, 0.5, frequency, 0.1, 600)
        pos.append(robot.getActualTCPPose()[0:3])
        accelerations.append(convertAccInTCP2Base(robot, robot.getActualToolAccelerometer()))

        accForce = (np.asarray(accelerations[-1]) - avgNoiseAcc) * payload
        inertiaCompensations.append(accForce)

        ftSensors.append(robot.getActualTCPForce()[0:3])    
        ftMagnitude.append(np.linalg.norm(ftSensors[-1]))




        ftCompensation = np.asarray(ftSensors[-1]) + np.asarray(inertiaCompensations[-1]) * [1,1,1]
        #print(np.asarray(ftSensors[-1]), " + ", np.asarray(inertiaCompensations[-1]) * [-1,1,-1], " = ", ftCompensation)
        ftCompensations.append(ftCompensation)

        sk = ((ftMagnitude[-1] - np.mean(ftMagnitude))**2 - (ftMagnitude[-1] - contactMean)**2)/(2*noiseSTD**2)
        cusumValues.append(np.maximum(0, cusumValues[-1] + sk))

        
        if cusumValues[-1] > threshold:
            break


        '''
        if time.time() - resetTime > 0.1:
            robot.zeroFtSensor()
            resetTime = time.time()
        '''
        i += 1
        if i >= nrOfPoints:
            i = 0
            traj = list(reversed(traj))
            time.sleep(1)
        count += 1
        diff = time.time() - startTime
        if diff < frequency:
            time.sleep(frequency - diff)

    robot.servoStop()
    accelerations = np.asarray(accelerations).T
    inertiaCompensations = np.asarray(inertiaCompensations).T
    ftSensors = np.asarray(ftSensors[1:]).T
    ftCompensations = np.asarray(ftCompensations).T
    cusumValues = np.asarray(cusumValues[1:])
    pos = np.asarray(pos).T
    ftMagnitude = np.asarray(ftMagnitude[1:])
    filteredFTMagnitude = np.asarray(filteredFTMagnitude[1:])
    plot(accelerations, ftSensors, inertiaCompensations, ftCompensations, cusumValues, pos, ftMagnitude, filteredFTMagnitude)



if __name__ == "__main__":
    main()
