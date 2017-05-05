import numpy as np
from multiprocess import Pool

# Boundary values for genes
GENE_MAX = 1000
GENE_MIN = -1000

class Generation(object):

    def __init__(
        self,
        gen_size,
        num_genes,
        num_organisms,
        elitism_thresh,
        crossover_thresh,
        mutation_thresh,
        fitness_func,
        num_jobs=None):

        self.gen_size = gen_size
        self.num_genes = num_genes
        self.num_organisms = num_organisms
        self.elitism_thresh = elitism_thresh
        self.crossover_thresh = crossover_thresh
        self.mutation_thresh = mutation_thresh
        self.fitness_func = fitness_func
        self.num_jobs = num_jobs

        self._organisms = np.around(np.random.rand(num_organisms, gen_size, \
            num_genes), 3)
        self._fitness_lists = np.zeros((num_organisms, gen_size))


    def evaluate_fitness(self):
        """
        Calculates fitness of all organisms in the generation and sorts by most
        fit.
        """

        pool = Pool(processes=self.num_jobs)
        fitness_output = np.array(pool.map(self.fitness_func, \
            np.transpose(self._organisms, (1,0,2))))
        self._fitness_lists = fitness_output.transpose().reshape((\
            self.num_organisms, self.gen_size))
        pool.close()
        pool.join()
        self._sort() # Make sure to sort at the end for fitness


    def get_zeroths(self):
        """
        Returns first organism and fitness.
        """

        best_organisms = []
        best_fitnesses = []
        for i in range(len(self._organisms)):
            best_organisms.append(self._organisms[i][0])
            best_fitnesses.append(self._fitness_lists[i][0])

        return best_organisms, best_fitnesses


    def evolve(self):
        """
        Method to evolve the generation of organisms.
        """

        for i in range(len(self._organisms)):

            # Fill a percentage of the next generation with elite organisms
            num_organisms_created = int(round(self.gen_size *
                    self.elitism_thresh))
            buf = self._organisms[i][:num_organisms_created].tolist()

            # Create rest of organisms with crossovers and mutations
            while (num_organisms_created < self.gen_size):

                # Randomly decide if the next organisms should be created
                # from a crossover_thresh
                organisms_to_create = self.gen_size - num_organisms_created
                crossover = 1 if np.random.rand()<=self.crossover_thresh else 0
                if crossover and organisms_to_create >= 2:

                    # Make two child organisms from tournament-selected parents
                    (parent_1, parent_2) = self._select_parents(i)
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
                    curr_organism = self._organisms[i][num_organisms_created]
                    if np.random.rand() <= self.mutation_thresh:
                        buf.append(self._mutate(curr_organism))
                    else:
                        buf.append(curr_organism)
                    num_organisms_created += 1

            # Sort current generation by fitness
            self._organisms[i] = np.asarray(buf)


    def _sort(self):
        """
        Sorts organisms by fitness.
        """
        for i in range(len(self._organisms)):
            order = self._fitness_lists[i].argsort()
            self._organisms[i] = self._organisms[i][order]
            self._fitness_lists[i] = self._fitness_lists[i][order]


    def _tournament_selection(self, organism_num):
        """
        A helper method used to select a random organism from the
        generation using a tournament selection algorithm.
        """
        # First, get some random indexes
        choices_idx = np.random.choice(self.gen_size, 5)

        # From those indexes, get the fitnesses and get the choice index of the
        # most fit
        min_idx = np.argmin(self._fitness_lists[organism_num][choices_idx])

        # Get the subset of organisms from the random sample. Then, use the
        # min_idx to get the most fit organism in the sample
        return self._organisms[organism_num][choices_idx][min_idx]


    def _select_parents(self, organism_num):
        """
        A helper method used to select two parents from the generation using a
        tournament selection algorithm.
        """
        return (self._tournament_selection(organism_num),
                self._tournament_selection(organism_num))


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
