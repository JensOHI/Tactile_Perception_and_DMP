import time
import keyboard
import os



class Demonstrate:
    def __init__(self, robot_, frequency_ = 1/500):
        self.robot = robot_
        self.frequency = frequency_
        self.tcps = []

    def saveDemonstration(self,filename):
        try:
            os.remove(filename+".dat")
        except OSError:
            pass
        with open(filename+".dat", 'x') as file:
            for content in self.tcps:
                string = ''
                for item in content:
                    string += str(item) + " "
                string = string[0:-2]
                string += "\n"
                file.write(string)



    def show(self, demonstrateTime = 5, saveAsFile = False, filename = "demonstration"):
        print("Demonstrate started! Please teach it now :D")
        self.tcps = []
        timeStart = time.time()
        self.robot.teachMode()
        while True:
            startTime = time.time()
            self.tcps.append(self.robot.getActualTCPPose())

            if time.time() - timeStart > demonstrateTime:
                break


            diffTime = time.time() - startTime
            if (diffTime < self.frequency):
                time.sleep(self.frequency - diffTime)

        self.robot.endTeachMode()
        print("Demonstration done!")
        if saveAsFile: self.saveDemonstration(filename)
        return self.tcps


