import time
import csv

import numpy as np

from .generation import Generation



class GeneticAlgorithm(object):

    def __init__(
        self,
        fitness_function,
        gen_size=1024,
        crossover_thresh=0.8,
        elitism_thresh=0.1,
        mutation_thresh=0.05,
        num_genes=4):

        self.gen_size = gen_size
        self.crossover_thresh = crossover_thresh
        self.elitism_thresh = elitism_thresh
        self.mutation_thresh = mutation_thresh
        self.num_genes = num_genes
        self.fitness_function = fitness_function
        self.max_generations = 10000

        self._generation = Generation(gen_size, num_genes)

    def train(self):

        print "Starting the training session."

        with open('logs/log_'+str(int(time.time()))+'.csv','wb') as file_obj:

            writer = csv.writer(file_obj, delimiter = ',')
            gen_idx = 0
            found = False

            while gen_idx < self.max_generations:

                # Evaluate the fitness for one generation
                self._generation.evaluate_fitness(self.fitness_function)

                # Print out the best
                print"Generation %d: %s" % (gen_idx, \
                        self._generation.chromosomes[0]), \
                        self._generation.fitnesses[0]

                # Save to the log
                writer.writerow([
                    gen_idx, \
                    self._generation.chromosomes[0], \
                    self._generation.fitnesses[0]
                ])
                file_obj.flush()

                if self._generation.fitnesses[0] < .25:
                    print "Most fit gene:", \
                            self._generation.chromosomes[0], \
                            self._generation.fitnesses[0]
                    found = True
                    break
                else:
                    self._evolve()

                gen_idx += 1

        if not found:
            print "Maximum generations reached without success."


    def _evolve(self):
        """
        Method to evolve the generation of chromosomes.
        """

        # Fill a percentage of the next generation with elite chromosomes
        num_chromosomes_created = int(round(self.gen_size * self.elitism_thresh))
        buf = self._generation.chromosomes[:num_chromosomes_created].tolist()

        # Create rest of chromsosomes with crossovers and mutations
        while (num_chromosomes_created < self.gen_size):

            # Randomly decide if the next chromosomes should be created
            # from a crossover_thresh
            chromosomes_to_create = self.gen_size - num_chromosomes_created
            if np.random.rand() <= self.crossover_thresh and chromosomes_to_create >= 2:

                # Create two child chromosomes from tournament-selected parents
                (parent_1, parent_2) = self._generation.select_parents()
                children = self._generation.crossover(parent_1, parent_2)

                # Random chance to mutate either child
                for child in children:
                    if np.random.rand() <= self.mutation_thresh:
                        buf.append(self._generation.mutate(child))
                    else:
                        buf.append(self._generation.mutate(child))
                num_chromosomes_created += 2

            # Directly move a past chromosome to the next generation with
            # a chance at mutation
            else:
                curr_chromosome = self._generation.chromosomes[num_chromosomes_created]
                if np.random.rand() <= self.mutation_thresh:
                    buf.append(self._generation.mutate(curr_chromosome))
                else:
                    buf.append(curr_chromosome)
                num_chromosomes_created += 1

        # Sort current generation by fitness
        self._generation.chromosomes = np.asarray(buf)
