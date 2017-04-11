#!usr/bin/env python


"""
Basic class that represents the chomosomes of our genetic algorithm.
"""


import random


# The number of genes that each organism has
NUM_GENES = 5


class Chromosome:
    """
    Holds the genes and fitness of an organism.
    """

    def __init__(self, genes=None):
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
        self.fitness = self.get_fitness()


    def crossover(self, other):
        """
        Mixes the two specified chromosomes, returning a new chromosome
        that is a result of a crossover of the two original chromosomes.

        other: second chromosome to crossover

        return: a chromosome that is a crossover between self and other
        """

        # Define the genes that will be crossovered
        g1 = self.genes
        g2 = other.genes

        # Define a random pivot point around which the crossover will occur
        crossover_point = random.randint(0, NUM_GENES-1)

        # Create the new crossovered gene and chromosome
        new_genes = g1[:crossover_point] + g2[crossover_point:]
        new_chromosome = Chromosome(new_genes)

        return new_chromosome


    def mutate(self, mutation_rate_multiplier=1.0):
        """
        Mutates the genes of the specified chromosome.

        mutation_rate_multiplier: alter the chance that each individual unit of
            a gene becomes mutated. Mutation chance per unit is
            mutation_rate_multiplier/num_units_per_gene
        """

        # Define the chance of each individual gene to be mutated
        mutation_rate = mutation_rate_multiplier/float(NUM_GENES)

        # Initialize what will be the final list of mutated genes
        mutated_genes = []

        # Iterate through all current genes
        for gene in self.genes:
            # Randomly decided whether or not to mutate each individual gene
            if random.uniform(0, 1) <= mutation_rate:
                gene *= random.uniform(0.8, 1.2)
            mutated_genes.append(round(gene, 3))

        # Replace the chromosome's genes with the mutated genes
        self.genes = mutated_genes


    def get_fitness(self):
        """
        Calculate the fitness of a specified chromosome.
        """

        # TODO: implement get_fitness function
        pass


if __name__ == '__main__':
    # Test basic functionality

    c1 = Chromosome()
    c2 = Chromosome()
    print "First generation:"
    print c1.genes
    print c2.genes, "\n"

    c3 = c1.crossover(c2)
    c4 = c1.crossover(c2)
    print "Second generation (after crossover):"
    print c3.genes
    print c4.genes, "\n"

    c3.mutate()
    c4.mutate()
    print "Second generation (after mutation):"
    print c3.genes
    print c4.genes
