import time
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



    def show(self, demonstrateTime = 5, saveAsFile = True, filename = "demonstration"):
        self.tcps = []
        timeStart = time.time()
        self.robot.teachMode()
        while time.time() - timeStart < demonstrateTime:
            startTime = time.time()

            self.tcps.append(self.robot.getActualTCPPose())

            diffTime = time.time() - startTime
            if (diffTime < self.frequency):
                time.sleep(self.frequency - diffTime)

        self.robot.endTeachMode()
        print("Demonstration done!")
        if saveAsFile: self.saveDemonstration(filename)
        return self.tcps


