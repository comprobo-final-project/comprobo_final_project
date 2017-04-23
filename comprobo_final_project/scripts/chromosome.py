#!usr/bin/env python


"""
Basic class that represents the chomosomes of our genetic algorithm.
"""


import random
import numpy as np

# The number of genes that each organism has
NUM_GENES = 6


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
                genes.append(round(random.uniform(0, 5), 3))

        # Define the chromosome's genes and fitness
        self.genes = genes
        self.natural_selection = supervisor
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
        new_chromosome_1 = Chromosome(new_genes_1, self.natural_selection)
        new_chromosome_2 = Chromosome(new_genes_2, self.natural_selection)

        return new_chromosome_1, new_chromosome_2


    def mutate(self, mutation_rate_multiplier=1.0):
        """
        Mutates the genes of the specified chromosome.

        mutation_rate_multiplier: alter the chance that each individual unit of
            a gene becomes mutated. Mutation chance per unit is
            mutation_rate_multiplier/num_units_per_gene
        """

        # Initialize what will be the final list of mutated genes
        mutated_genes = []

        for gene in self.genes:
            mutated_genes.append(gene)

        # Select a random gene and multiply it with a random value
        idx = random.randint(0, len(self.genes) - 1)
        mutated_genes[idx] *= random.uniform(0.5, 2)

        # Create new chromosome with genes from the mutated genes
        return Chromosome(mutated_genes, self.natural_selection)

    def get_fitness(self):
        """
        Calculate the fitness of a specified chromosome.
        """

        self.natural_selection.use_genes(self.genes)
        xpos, ypos = self.natural_selection.run()
        fitness = np.sqrt(xpos**2 + ypos**2)
        self.natural_selection.reset()
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
