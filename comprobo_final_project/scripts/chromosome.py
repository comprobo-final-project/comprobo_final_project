#!usr/bin/env python


"""
Basic class that represents the chomosomes of our genetic algorithm.
"""


import random
import numpy as np

# The number of genes that each organism has
NUM_GENES = 4

# Boundary values for genes
GENE_MAX = 10000
GENE_MIN = -10000


class Chromosome:
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

        # Define the chromosome's genes and fitness
        self.genes = genes
        self.supervisor = supervisor
        self.fitness = self.get_fitness()


    def crossover(self, other):
        """
        Mixes the two specified chromosomes, returning two new chromosomes
        that are a result of a crossover of the two original chromosomes.

        other: second chromosome to crossover

        return: two chromosomes that are crossovers between self and other
        """

        # Define the genes that will be crossovered
        g1 = self.genes
        g2 = other.genes

        # Define a random pivot point around which the crossover will occur
        crossover_point = random.randint(0, NUM_GENES-1)

        # Create the new crossovered genes and chromosome
        new_genes_1 = g1[:crossover_point] + g2[crossover_point:]
        new_genes_2 = g2[:crossover_point] + g1[crossover_point:]
        new_chromosome_1 = Chromosome(new_genes_1, self.supervisor)
        new_chromosome_2 = Chromosome(new_genes_2, self.supervisor)

        return new_chromosome_1, new_chromosome_2


    def mutate(self):
        """
        Mutates a single random gene of the specified chromosome.
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

        # Create new chromosome with genes from the mutated genes
        return Chromosome(mutated_genes, self.supervisor)


    def get_fitness(self):
        """
        Calculate the fitness of a specified chromosome.
        """

        # Apply current chromosome's genes to the supervisor
        self.supervisor.use_genes(self.genes)

        # Calculate fitness
        poses = self.supervisor.run() # all poses
        distances = [np.sqrt(pose.x**2 + pose.y**2) \
                for pose in poses] # all distances from goal
        fitness = np.mean(distances) # average distance from goal

        # Reset the supervisor to accept new genes
        self.supervisor.reset()

        return fitness


if __name__ == '__main__':

    # Test basic functionality

    c1 = Chromosome()
    c2 = Chromosome()
    print "First generation:"
    print c1.genes
    print c2.genes, "\n"

    c3, c4 = c1.crossover(c2)
    print "Second generation (after crossover):"
    print c3.genes
    print c4.genes, "\n"

    c3.mutate()
    c4.mutate()
    print "Second generation (after mutation):"
    print c3.genes
    print c4.genes
