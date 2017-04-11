#!usr/bin/env python

import Chromosome
from random import choice, random, randint

"""
CompRobo Spring 2017

population script for our simplified task, one Neato moving to a goal

"""


class Population:
    """
    A class representing a population for a genetic algorithm simulation.
    """

    def __init__(self, size=1024, crossover=0.8, elitism=0.1, mutation=0.05):
        self.elitism = elitism
        self.mutation = mutation
        self.crossover = crossover
        self.tournament_size = int(size * 0.005)

        pop = []
        for i in range(size): pop.append(Chromosome.gen_random())
        self.population = list(sorted(buf, key=lambda x: x.fitness))


    def tournament_selection(self):
        """
        A helper method used to select a random chromosome from the
        population using a tournament selection algorithm.
        """

        best = choice(self.population)
        for i in range(self.tournament_size):
            cont = choice(self.population)
            if (cont.fitness < best.fitness): best = cont

        return best


    def select_parents(self):
        """
        A helper method used to select two parents from the population using a
        tournament selection algorithm.
        """

        return (self._tournament_selection(), self._tournament_selection())


    def evolve(self):
        """
        Method to evolve the population of chromosomes.
        """

        size = len(self.population)
        idx = int(round(size * self.elitism))
        buf = self.population[:idx]

        while (idx < size):
            if random() <= self.crossover:
                (p1, p2) = self._selectParents()
                children = p1.mate(p2)
                for c in children:
                    if random() <= self.mutation:
                        buf.append(c.mutate())
                    else:
                        buf.append(c)
                idx += 2
            else:
                if random() <= self.mutation:
                    buf.append(self.population[idx].mutate())
                else:
                    buf.append(self.population[idx])
                idx += 1

        self.population = list(sorted(buf, key=lambda x: x.fitness))
