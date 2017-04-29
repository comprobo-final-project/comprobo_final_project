import numpy as np

# Boundary values for genes
GENE_MAX = 10000
GENE_MIN = -10000

class Generation(object):

    def __init__(
        self,
        gen_size,
        num_genes,
        elitism_thresh,
        crossover_thresh,
        mutation_thresh,
        fitness_func):

        self.gen_size = gen_size
        self.num_genes = num_genes
        self.elitism_thresh = elitism_thresh
        self.crossover_thresh = crossover_thresh
        self.mutation_thresh = mutation_thresh
        self.fitness_func = fitness_func

        self._chromosomes = np.random.rand(gen_size, num_genes)
        self._fitnesses = np.zeros(gen_size)


    def evaluate_fitness(self):
        self._fitnesses = np.apply_along_axis(self.fitness_func, 1, \
            self._chromosomes)
        self._sort() # Make sure to sort at the end for fitness


    def get_zeroth(self):
        return self._chromosomes[0], self._fitnesses[0]


    def evolve(self):
        """
        Method to evolve the generation of chromosomes.
        """

        # Fill a percentage of the next generation with elite chromosomes
        num_chromosomes_created = int(round(self.gen_size * self.elitism_thresh))
        buf = self._chromosomes[:num_chromosomes_created].tolist()

        # Create rest of chromsosomes with crossovers and mutations
        while (num_chromosomes_created < self.gen_size):

            # Randomly decide if the next chromosomes should be created
            # from a crossover_thresh
            chromosomes_to_create = self.gen_size - num_chromosomes_created
            if np.random.rand() <= self.crossover_thresh and chromosomes_to_create >= 2:

                # Create two child chromosomes from tournament-selected parents
                (parent_1, parent_2) = self._select_parents()
                children = self._crossover(parent_1, parent_2)

                # Random chance to mutate either child
                for child in children:
                    if np.random.rand() <= self.mutation_thresh:
                        buf.append(self._mutate(child))
                    else:
                        buf.append(self._mutate(child))
                num_chromosomes_created += 2

            # Directly move a past chromosome to the next generation with
            # a chance at mutation
            else:
                curr_chromosome = self._chromosomes[num_chromosomes_created]
                if np.random.rand() <= self.mutation_thresh:
                    buf.append(self._mutate(curr_chromosome))
                else:
                    buf.append(curr_chromosome)
                num_chromosomes_created += 1

        # Sort current generation by fitness
        self._chromosomes = np.asarray(buf)


    def _sort(self):
        order = self._fitnesses.argsort()
        self._chromosomes = self._chromosomes[order]
        self._fitnesses = self._fitnesses[order]


    def _tournament_selection(self):
        """
        A helper method used to select a random chromosome from the
        generation using a tournament selection algorithm.
        """
        choices = np.random.choice(self.gen_size, 5)
        choice = np.argmin(self._fitnesses[choices])
        return self._chromosomes[choices][choice]


    def _select_parents(self):
        """
        A helper method used to select two parents from the generation using a
        tournament selection algorithm.
        """
        return (self._tournament_selection(), self._tournament_selection())


    def _crossover(self, chromosome_1, chromosome_2):
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


    def _mutate(self, chromosome):
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
