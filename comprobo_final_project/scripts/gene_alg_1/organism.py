#!usr/bin/env python


"""
Basic class that represents the chomosomes of our genetic algorithm.
"""


import random
import numpy as np
from scipy import stats

# The number of genes that each organism has
NUM_GENES = 12


# Boundary values for genes
GENE_MAX = 10000
GENE_MIN = -10000


class Organism:
    """
    Holds the genes and fitness of an organism.
    """

    def __init__(self, genes=None, supervisor=None):
        """
        Initializes the gene and fitness of an organism.
        """

        # Generate a random set of genes if genes are not already defined
        if genes == None:
            genes = []
            for x in range(NUM_GENES):
                genes.append(round(random.uniform(-5, 5), 3))

        # Define the organism's genes and fitness
        self.genes = genes
        self.supervisor = supervisor
        self.fitness = self.get_fitness_collinear()


    def crossover(self, other):
        """
        Mixes the two specified organisms, returning two new organisms
        that are a result of a crossover of the two original organisms.

        other: second organism to crossover

        return: two organisms that are crossovers between self and other
        """

        # Define the genes that will be crossovered
        g1 = self.genes
        g2 = other.genes

        # Define a random pivot point around which the crossover will occur
        crossover_point = random.randint(0, NUM_GENES-1)

        # Create the new crossovered genes and organism
        new_genes_1 = g1[:crossover_point] + g2[crossover_point:]
        new_genes_2 = g2[:crossover_point] + g1[crossover_point:]
        new_organism_1 = Organism(new_genes_1, self.supervisor)
        new_organism_2 = Organism(new_genes_2, self.supervisor)

        return new_organism_1, new_organism_2


    def mutate(self):
        """
        Mutates a single random gene of the specified organism.
        """

        # Initialize what will be the final list of mutated genes
        mutated_genes = self.genes

        # Select a random gene and multiply it with a random value
        index_to_mutate = random.randint(0, len(self.genes) - 1)
        mutated_genes[index_to_mutate] *= random.uniform(0.5, 2)

        # Clip and round all genes
        mutated_genes[index_to_mutate] = np.clip(mutated_genes[index_to_mutate],
                GENE_MIN, GENE_MAX)
        mutated_genes = [round(gene, 3) for gene in mutated_genes]

        # Create new organism with genes from the mutated genes
        return Organism(mutated_genes, self.supervisor)


    def get_fitness_simple(self):
        """
        Calculates fitness of a specified organism for simple task
        """

        # Apply current organism's genes to the supervisor
        self.supervisor.use_genes(self.genes)

        # Calculate fitness
        positions = self.supervisor.run() # all positions
        distances = [np.sqrt(position.x**2 + position.y**2) \
                for position in positions] # all distances from goal
        fitness = np.mean(distances) # average distance from goal

        # Reset the supervisor to accept new genes
        self.supervisor.reset()

        return fitness

    def get_fitness_collinear(self):
        """
        Calculate the fitness of a specified organism for collinear task
        """

        fitness = []
        for i in range(3):
            self.supervisor.use_genes(self.genes)

            positions = self.supervisor.run()
            end = positions[-1]
            # print "ROBOT END", end[0].x, end[0].y, end[1].x, end[1].y, end[2].x, end[2].y
            r_values = []

            for position in positions:
                x = []
                y = []
                for robot in position:
                    x.append(robot.x)
                    y.append(robot.y)

                _, _, r_value, _, _ = stats.linregress(zip(x,y))
                r_values.append(r_value**2)

            final_value = np.mean(r_values)
            fitness.append(final_value)
            self.supervisor.reset()
            print

        overall_fitness = np.mean(fitness)
        print overall_fitness
        return overall_fitness


if __name__ == '__main__':

    # Test basic functionality
    from supervisor import Supervisor
    test = Supervisor()
    c1 = Organism([0.472, -2.264, -0.48, 3.669, -3.244, -0.746, -4.727, 4.552, 3.9, 3.054, -1.903, -3.669], test)
