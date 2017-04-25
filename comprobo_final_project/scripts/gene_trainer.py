#!/usr/bin/env python

"""
The main script of this project, runs a genetic algorithm to find the optimal
parameters to achieve a solution.
"""

import time
import csv

from population import Population
from supervisor import Supervisor
from robot_controller import RobotController
from simulator.robot import Robot


class GeneTrainer(object):
    """
    main class that holds the entirety of the genetic algorithm, which will
    optimize parameters towards a goal
    """

    def __init__(self):
        self.supervisor = Supervisor()
        self.population = Population(size=100, crossover=0.8, elitism=0.1, \
                mutation=0.5, supervisor=self.supervisor)

        self.max_generations = 16384


    def train(self):

        print "Starting the training session."

        with open('logs/log_' + str(int(time.time())) + '.csv','wb') as file_obj:
            writer = csv.writer(file_obj, delimiter = ',')
            generation = 0
            found = False
            while generation < self.max_generations:
                print"Generation %d: %s" % (generation, \
                        self.population.generation[0].genes), \
                        self.population.generation[0].fitness

                # Save to the log
                writer.writerow([
                    generation,
                    self.population.generations[0].genes,
                    self.population.generations[0].fitness
                ])
                file_obj.flush()

                if self.population.population[0].fitness < 0.05:
                    print "Most fit gene:", self.population.generation[0].genes, \
                            self.population.generation[0].fitness
                    found = True
                    break
                else:
                    self.population.evolve()

                generation += 1

        if not found:
            print "Maximum generations reached without success."


if __name__ == '__main__':
    trainer = GeneTrainer()
    trainer.train()
