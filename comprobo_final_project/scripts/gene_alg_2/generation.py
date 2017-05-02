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

        self._organisms = np.around(np.random.rand(gen_size, num_genes), 3)
        self._fitnesses = np.zeros(gen_size)


    def evaluate_fitness(self):
        """
        Calculates fitness of all organisms in the generation and sorts by most
        fit.
        """
        self._fitnesses = np.apply_along_axis(self.fitness_func, 1, \
            self._organisms)
        self._sort() # Make sure to sort at the end for fitness


    def get_zeroth(self):
        """
        Returns first organism and fitness.
        """
        return self._organisms[0], self._fitnesses[0]


    def evolve(self):
        """
        Method to evolve the generation of organisms.
        """

        # Fill a percentage of the next generation with elite organisms
        num_organisms_created = int(round(self.gen_size * self.elitism_thresh))
        buf = self._organisms[:num_organisms_created].tolist()

        # Create rest of organisms with crossovers and mutations
        while (num_organisms_created < self.gen_size):

            # Randomly decide if the next organisms should be created
            # from a crossover_thresh
            organisms_to_create = self.gen_size - num_organisms_created
            crossover = 1 if np.random.rand() <= self.crossover_thresh else 0
            if crossover and organisms_to_create >= 2:

                # Create two child organisms from tournament-selected parents
                (parent_1, parent_2) = self._select_parents()
                children = self._crossover(parent_1, parent_2)

                # Random chance to mutate either child
                for child in children:
                    if np.random.rand() <= self.mutation_thresh:
                        buf.append(self._mutate(child))
                    else:
                        buf.append(self._mutate(child))
                num_organisms_created += 2

            # Directly move a past organism to the next generation with
            # a chance at mutation
            else:
                curr_organism = self._organisms[num_organisms_created]
                if np.random.rand() <= self.mutation_thresh:
                    buf.append(self._mutate(curr_organism))
                else:
                    buf.append(curr_organism)
                num_organisms_created += 1

        # Sort current generation by fitness
        self._organisms = np.asarray(buf)


    def _sort(self):
        """
        Sorts organisms by fitness.
        """
        order = self._fitnesses.argsort()
        self._organisms = self._organisms[order]
        self._fitnesses = self._fitnesses[order]


    def _tournament_selection(self):
        """
        A helper method used to select a random organism from the
        generation using a tournament selection algorithm.
        """
        # First, get some random indexes
        choices_idx = np.random.choice(self.gen_size, 5)

        # From those indexes, get the fitnesses and get the choice index of the
        # most fit
        min_idx = np.argmin(self._fitnesses[choices_idx])

        # Get the subset of organisms from the random sample. Then, use the
        # min_idx to get the most fit organism in the sample
        return self._organisms[choices_idx][min_idx]


    def _select_parents(self):
        """
        A helper method used to select two parents from the generation using a
        tournament selection algorithm.
        """
        return (self._tournament_selection(), self._tournament_selection())


    def _crossover(self, organism_1, organism_2):
        """
        Mixes the two specified organisms, returning two new organisms
        that are a result of a crossover of the two original organisms.

        organism_1, organism_2: organisms to crossover
        return: two organisms that are crossed over
        """

        org_1 = organism_1[:]
        org_2 = organism_2[:]

        # Define a random pivot point around which the crossover will occur
        pivot = np.random.randint(0, self.num_genes-1)

        # Create the new crossovered genes and organism
        right_genes = org_2[:pivot].copy()
        org_2[:pivot], org_1[:pivot] = org_1[:pivot], right_genes

        return org_1, org_2


    def _mutate(self, organism):
        """
        Mutates a single random gene of the specified organism.
        """

        # Don't modify existing organism
        _organism = organism[:]

        # Select a random gene and multiply it with a random value
        index_to_mutate = np.random.randint(0, len(_organism) - 1)
        _organism[index_to_mutate] *= np.random.uniform(0.5, 2)

        # Clip and round all genes
        _organism[index_to_mutate] = np.clip(_organism[index_to_mutate],
                GENE_MIN, GENE_MAX)
        _organism = [np.round(gene, 3) for gene in _organism]

        # Create new organism with genes from the mutated genes
        return _organism
