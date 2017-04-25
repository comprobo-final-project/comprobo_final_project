#!/usr/bin/env python


"""
The Supervisor class is needed to create a single package that can be sent to all Chromosomes to allow them
to each calculate their own fitness using the simulation components in this class
"""


from simulator.simulator import Simulator
from simulator.robot import Robot
from robot_controller import RobotController
import numpy as np

class Supervisor(object):
    """
    evaluator class that holds the components needed for simulation and
    determining fitness
    """

    def __init__(self):

        self.robot = Robot()
        self.robot.pose.position.x = np.random.randint(1,10)
        self.robot.pose.position.y = np.random.randint(1,10)
        self.sim = Simulator(self.robot)
        self.robo_control = RobotController(self.robot)


    def use_genes(self, genes):
        """
        receives this current iteration of the genes
        """

        self.robo_control.set_genes(genes)


    def reset(self):
        """
        resets the simulation for the next usage
        """

        self.robot.pose.position.x = np.random.randint(1,10)
        self.robot.pose.position.y = np.random.randint(1,10)
        self.robot.pose.velocity.x = 0.0
        self.robot.pose.velocity.y = 0.0
        self.robot.pose.orientation.z = np.random.randint(1,10)


    def run(self):
        """
        main run function
        """
        xpos, ypos = self.robo_control.run(30)

        return xpos, ypos


if __name__ == '__main__':

    node = Supervisor()
    genes = [1.0,2.0,3.0,4.0,5.0,6.0]
    node.use_genes(genes)
    node.run()
