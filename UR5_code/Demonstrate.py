import time
import keyboard



class Demonstrate:
    def __init__(self, robot_, frequency_):
        self.robot = robot_
        self.frequency = frequency_
        self.tcps = []


    def show(self):
        self.tcps = []
        while True:
            startTime = time.time()
            self.tcps.append(self.robot.getActualTCPPose())

            if keyboard.is_pressed('d'):
                break

            diffTime = time.time() - startTime
            if (diffTime < self.frequency):
                time.sleep(diffTime)

        print("Demonstration done!")


