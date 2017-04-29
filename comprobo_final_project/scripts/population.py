#!usr/bin/env python


"""
CompRobo Spring 2017

generation script for our simplified task, one Neato moving to a goal

"""


from organism import Organism
from random import choice, random, randint


class Population:
    """
    A class representing a generation for a genetic algorithm simulation.
    """

    def __init__(self, size=1024, crossover=0.8, elitism=0.1, mutation=0.05,
            supervisor=None):

        self.elitism = elitism
        self.mutation = mutation
        self.crossover = crossover
        self.tournament_size = 3

        pop = []
        for i in range(size): pop.append(Organism(genes=None, supervisor=supervisor))

        # population class holds a generation of the population, which evolves over time
        self.generations = list(sorted(pop, key=lambda x: x.fitness, reverse=True))



    def tournament_selection(self):
        """
        A helper method used to select a random chromosome from the
        generation using a tournament selection algorithm.
        """

        best = choice(self.generations)
        for i in range(self.tournament_size):
            cont = choice(self.generations)
            if (cont.fitness < best.fitness): best = cont

        return best


    def select_parents(self):
        """
        A helper method used to select two parents from the generation using a
        tournament selection algorithm.
        """

        return (self.tournament_selection(), self.tournament_selection())


    def evolve(self):
        """
        Method to evolve the generation of chromosomes.
        """

        size = len(self.generations)
        idx = int(round(size * self.elitism))
        buf = self.generations[:idx]

        while (idx < size):
            if random() <= self.crossover:
                (p1, p2) = self.select_parents()
                children = p1.crossover(p2)
                for c in children:
                    if random() <= self.mutation:
                        buf.append(c.mutate())
                    else:
                        buf.append(c)
                idx += 2
            else:
                if random() <= self.mutation:
                    buf.append(self.generations[idx].mutate())
                else:
                    buf.append(self.generations[idx])
                idx += 1

        self.generations = list(sorted(buf, key=lambda x: x.fitness, reverse=True))
