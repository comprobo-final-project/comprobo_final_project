import numpy as np

# Boundary values for genes
GENE_MAX = 10000
GENE_MIN = -10000

class Generation(object):

    def __init__(self, size, num_genes):
        self.size = size
        self.num_genes = num_genes
        self.chromosomes = np.random.rand(size, num_genes)
        self.fitnesses = np.zeros(size)


    def evaluate_fitness(self, fitness_func):
        self.fitnesses = np.apply_along_axis(fitness_func, 1, self.chromosomes)
        self.sort()


    def sort(self):
        order = self.fitnesses.argsort()
        self.chromosomes = self.chromosomes[order]
        self.fitnesses = self.fitnesses[order]


    def tournament_selection(self):
        """
        A helper method used to select a random chromosome from the
        generation using a tournament selection algorithm.
        """
        choices = np.random.choice(self.size, 5)
        choice = np.argmin(self.fitnesses[choices])
        return self.chromosomes[choices][choice]


    def select_parents(self):
        """
        A helper method used to select two parents from the generation using a
        tournament selection algorithm.
        """
        return (self.tournament_selection(), self.tournament_selection())


    def crossover(self, chromosome_1, chromosome_2):
        """
        Mixes the two specified chromosomes, returning two new chromosomes
        that are a result of a crossover of the two original chromosomes.

        other: second chromosome to crossover

        return: two chromosomes that are crossovers between self and other
        """

        # Define a random pivot point around which the crossover will occur
        crossover_point = np.random.randint(0, self.num_genes-1)

        # Create the new crossovered genes and chromosome
        new_chromosome_1 = np.concatenate((chromosome_1[:crossover_point], chromosome_2[crossover_point:]))
        new_chromosome_2 = np.concatenate((chromosome_2[:crossover_point], chromosome_1[crossover_point:]))

        return new_chromosome_1, new_chromosome_2


    def mutate(self, chromosome):
        """
        Mutates a single random gene of the specified chromosome.
        """

        # Don't modify existing chromosome
        _chromosome = chromosome[:]

        # Select a random gene and multiply it with a random value
        index_to_mutate = np.random.randint(0, len(_chromosome) - 1)
        _chromosome[index_to_mutate] *= np.random.uniform(0.5, 2)

        # Clip and round all genes
        _chromosome[index_to_mutate] = np.clip(_chromosome[index_to_mutate],
                GENE_MIN, GENE_MAX)
        _chromosome = [np.round(gene, 3) for gene in _chromosome]

        # Create new chromosome with genes from the mutated genes
        return _chromosome
