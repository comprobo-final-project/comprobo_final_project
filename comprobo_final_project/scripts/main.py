#!/usr/bin/env python

"""
The main script of this project, runs a genetic algorithm to find the optimal
parameters to achieve a solution.
"""

from models.robot import Robot
from population import Population
from supervisor import Supervisor

from simulator.simulator import Simulator
from simulator.robot import Robot
from robot_controller import RobotController


class Genetics(object):
    """
    main class that holds the entirety of the genetic algorithm, which will
    optimize parameters towards a goal
    """

    def __init__(self):
        self.supervisor = Supervisor()
        self.population = Population(size=100, crossover=0.8, elitism=0.1, mutation=0.5, supervisor=self.supervisor)

        self.maxGenerations = 16384


    def run(self):
        """
        main run function
        """

        print "running genetics"
        generation = 0
        found = False
        while generation < self.maxGenerations:
            print"Generation %d: %s" % (generation, self.population.generation[0].genes), self.population.generation[0].fitness

            if self.population.population[0].fitness < 0.05:
                print "Most fit gene:", self.population.generation[0].genes, self.population.generation[0].fitness
                found = True
                break
            else:
                self.population.evolve()

            generation += 1

        if not found:
            print "Maximum generations reached without success."


if __name__ == '__main__':
    node = Genetics()
    node.run()
