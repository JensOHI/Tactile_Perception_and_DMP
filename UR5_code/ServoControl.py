import time


class ServoControl:
    def __init__(self, robot):
        self.frequency = 1/500

        self.robot = robot

    def run(self, event, positionalTrajectory, scaling = 1, velocity = 0.5, acceleration = 0.5, lookAheadTime = 0.1, gain = 300):
        print("Running DMP")
        
        lastTCPPose = self.robot.getActualTCPPose()

        
        pose = positionalTrajectory[0]
        #print(pose, type(pose))
        self.robot.moveL(pose)
        pointsTouched = 0
        event.set()
        for point in positionalTrajectory:
            startTime = time.time()

            pose = point
            self.robot.servoL(pose, velocity, acceleration, self.frequency/2, lookAheadTime, gain)
            pointsTouched += 1

            if event.is_set():
                break

            
            
            diff = time.time() - startTime
            if(diff < self.frequency):
                time.sleep(self.frequency - diff)
        
        self.robot.servoStop()
