"""
Basic class that represents the chomosomes of our genetic algorithm.
"""


from random import randint


class Chromosome:
    """
    Holds the genes and fitness of an organism.
    """

    def __init__(self, genes=None):
        """
        Initializes the gene and fitness of an organism.
        """

        self.genes = genes
        self.fitness = self.get_fitness()


    def get_fitness(self):
        """
        Calculate the fitness of a specified chromosome.
        """

        pass


def crossover(c1, c2):
    """
    Mixes the two specified chromosomes, returning a new chromosome
    that is a result of a crossover of the two original chromosomes.

    c1: first chromosome to crossover
    c2: second chromosome to crossover

    return: a chromosome that is a crossover between c1 and c2
    """

    # Define the genes that will be crossovered
    g1 = c1.genes
    g2 = c2.genes

    # Define a random pivot point around which the crossover will occur
    crossover_point = randint(0, len(c1.genes)-1)

    # Create the new crossovered gene and chromosome
    new_genes = c1.genes[crossover_point:] + c2.genes[:crossover_point]
    new_chromosome = Chromosome(new_genes)

    return new_chromosome


if __name__ == '__main__':
    c1 = Chromosome([0, 1, 2, 3, 4])
    c2 = Chromosome([4, 3, 2, 1, 0])
    c3 = crossover(c1, c2)
    print c3.genes
