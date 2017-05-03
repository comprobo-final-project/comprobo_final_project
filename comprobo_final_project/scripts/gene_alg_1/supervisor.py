#!/usr/bin/env python


"""
The Supervisor class is needed to create a single package that can be sent to all Chromosomes to allow them
to each calculate their own fitness using the simulation components in this class
"""


from ..simulator.robot import Robot
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
            robot = Robot(noise=0.0)
            robot.pose.position.x = np.random.randint(1,10)
            robot.pose.position.y = np.random.randint(1,10)
            self.robots.append(robot)
            # print "ROBOT START", robot.pose.position.x, robot.pose.position.y

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
        # Give the robot a random position r meters away from the goal
        robot.set_random_position(r=5.0)
        # print "ROBOT START", robot.pose.position.x, robot.pose.position.y

        # Give the robot a random direction
        robot.set_random_direction()


    def run(self):
        """
        main run function
        """

        return self.robot_controller.run(20)


if __name__ == '__main__':

    node = Supervisor()
    genes = [1.0, 2.0, 3.0, 4.0]
    node.use_genes(genes)
    node.run()