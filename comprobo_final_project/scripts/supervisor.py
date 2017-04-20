#!usr/bin/env python

"""
Evaluator class that acts as the bridge between simulation and evolution
"""

from simulator.simulator import Simulator
from simulator.robot import Robot
from robot_controller import RobotController

class Supervisor(object):
    """
    Evaluating the fitness of a given chromosome
    """

    def __init__(self):

        self.robot = Robot()
        robot.pose.position.x = 3.0
        robot.pose.position.y = 5.0
        self.sim = Simulator(self.robot)
        self.robo_control = RobotController()


    def use_genes(self, genes):
        """
        receives this current iteration of the genes
        """
        self.robo_control.set_genes(genes)


    def run(self):
        """
        main loop of the Evaluator
        """
        fitness = self.sim.run()

        return fitness


if __name__=="__main__":
    self.eval = Supervisor()
    self.eval.use_genes([0.0,1.0,2.0,3.0,4.0,5.0])
    self.eval.run()
