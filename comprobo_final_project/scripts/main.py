#!/usr/bin/env python

"""
The main script of this project, runs a genetic algorithm to find the optimal parameters
to achieve a solution.
"""

import rospy
from models import robot
from robot_controller import RobotController
from population import Population


class Genetics(object):
    """
    main class that holds the entirety of the genetic algorithm, which will optimize
    parameters towards a goal
    """

    def __init__(self):
        rospy.init_node('main')
        self.population = Population(size=2048, crossover=0.8, elitism=0.1, mutation=0.3)
        self.maxGenerations = 16384

        #ROS Subscribers?


    def run():
         def run(self):
        """
        main run function
        """

        print "running genetics"
        r = rospy.Rate(10)
        generation = 0

        while not rospy.is_shutdown() and generation < self.maxGenerations:
    		print("Generation %d: %s" % (i, pop.population[0].gene))
    		if pop.population[0].fitness < 0.1:
                break
    		else:
                self.population.evolve()

        	r.sleep()

        print("Maximum generations reached without success.")


if __name__ == '__main__':
    node = Genetics()
    node.run()
