#!/usr/bin/env python


"""
The Supervisor class is needed to create a single package that can be sent to all Chromosomes to allow them
to each calculate their own fitness using the simulation components in this class
"""


from simulator.robot import Robot
from robot_controller import RobotController
import numpy as np

class Supervisor(object):
    """
    evaluator class that holds the components needed for simulation and
    determining fitness
    """

    def __init__(self, number):

        self.robots = []
        for i in range(number):
            robot = Robot()
            robot.pose.position.x = np.random.randint(1,10)
            robot.pose.position.y = np.random.randint(1,10)
            self.robots.append(robot)

        self.sim = Simulator(self.robots)
        self.robot_controller = RobotController(self.robots)



    def use_genes(self, genes):
        """
        receives this current iteration of the genes
        """

        self.robot_controller.set_genes(genes)


    def reset(self):
        """
        resets the simulation for the next usage
        """
        for robot in self.robots:
            self.reset_robot(robot)


    def reset_robot(self, robot):
        """
        resets each robot's coordinates for next iteration
        """

        robot.pose.position.x = np.random.randint(1,10)
        robot.pose.position.y = np.random.randint(1,10)
        robot.pose.velocity.x = 0.0
        robot.pose.velocity.y = 0.0
        robot.pose.orientation.z = 0.0


    def run(self):
        """
        main run function
        """
        xpos1, ypos1, o1, xpos2, ypos2, o2, xpos3, ypos3, o3 = self.robot_controller.run(1)

        return xpos1, ypos1, xpos2, ypos2, xpos3, ypos3


if __name__ == '__main__':

    node = Supervisor()
    genes = [1.0,2.0,3.0,4.0,5.0,6.0]
    node.use_genes(genes)
    node.run()
