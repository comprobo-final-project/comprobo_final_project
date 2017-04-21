#!/usr/bin/env python

"""
The main script of this project, runs a genetic algorithm to find the optimal parameters
to achieve a solution.
"""

from models import robot
from population import Population
from supervisor import Supervisor

from simulator.simulator import Simulator
from simulator.robot import Robot
from robot_controller import RobotController


class Genetics(object):
    """
    main class that holds the entirety of the genetic algorithm, which will optimize
    parameters towards a goal
    """

    def __init__(self):
        self.supervisor = Supervisor
        self.population = Population(size=2048, crossover=0.8, elitism=0.1, mutation=0.3, self.supervisor)
        self.maxGenerations = 16384


    def run(self):
        """
        main run function
        """

        print "running genetics"
        generation = 0

        while generation < self.maxGenerations:
    		print("Generation %d: %s" % (generation, pop.population[0].gene))
    		if pop.population[0].fitness < 0.1:
                print "Most fit gene:", pop.population[0].gene
                break
    		else:
                self.population.evolve()

            generation += 1

        print("Maximum generations reached without success.")


if __name__ == '__main__':
    node = Genetics()
    node.run()
