from __future__ import division, print_function
from DMP.dmp_position import PositionDMP
import numpy as np


class DMP:
    def __init__(self, filename = "demonstration.dat", N = 50, alpha = 48):
        self.demo = np.loadtxt("demonstration.dat", delimiter=" ", skiprows=1)

        self.dmp = PositionDMP(n_bfs=N, alpha=alpha)

        self.tau = 0.002 * len(self.demo)
        self.t = np.arange(0, self.tau, 0.002)
        self.demo_p = self.demo[:, 0:3]
        self.dmp.train(self.demo_p, self.t, self.tau)

        self.dmp_p = []
        self.dmp_dp = []
        self.dmp_ddp = []
    
    def setGoal(self, goal):
        self.dmp.gp = goal
    
    def setObstacle(self, obstacle):
        self.dmp.obstacles.append(obstacle)

    def rollout(self, scaleTau = 1):
        self.tNew = np.arange(0, self.tau*scaleTau,0.002)
        self.dmp_p, self.dmp_dp, self.dmp_ddp = self.dmp.rollout(self.tNew, self.tau*scaleTau)
        return self.dmp_p, self.dmp_dp, self.dmp_ddp

    def plotTrajectory(self):
        self.dmp.plot2DDMP(self.demo_p, self.dmp_p,self.t, self.tNew)
        self.dmp.plot3DDMP(self.demo_p,self.dmp_p)


