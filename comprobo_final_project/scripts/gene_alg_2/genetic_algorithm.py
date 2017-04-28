import time
import csv

import numpy as np
import pandas as pd

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

        self._generation = pd.DataFrame({
            'fitness': np.zeros(shape=gen_size)
        })
        self._generation['chromosome'] = pd.Series(list(np.random.rand(gen_size, num_genes)))

    def train(self):

        print "Starting the training session."

        with open('logs/log_'+str(int(time.time()))+'.csv','wb') as file_obj:

            writer = csv.writer(file_obj, delimiter = ',')
            gen_idx = 0
            found = False

            while gen_idx < self.max_generations:

                # Evaluate the fitness for one generation
                self._evaluate_fitness()

                # Print out the best
                print"Generation %d: %s" % (gen_idx, \
                        self._generation.iloc[0]['chromosome']), \
                        self._generation.iloc[0]['fitness']

                # Save to the log
                writer.writerow([
                    gen_idx, \
                    self._generation.iloc[0]['chromosome'], \
                    self._generation.iloc[0]['fitness']
                ])
                file_obj.flush()

                if self._generation.iloc[0]['fitness'] < .25:
                    print "Most fit gene:", \
                            self._generation.iloc[0]['chromosome'], \
                            self._generation.iloc[0]['fitness']
                    found = True
                    break
                else:
                    self._evolve()

                gen_idx += 1

        if not found:
            print "Maximum generations reached without success."


    def _evaluate_fitness(self):
        self._generation['fitness'] = self._generation['chromosome'].apply(self.fitness_function)
        self._generation.sort(columns='fitness', ascending=True)


    def _evolve(self):
        """
        Method to evolve the generation of chromosomes.
        """

        idx = int(np.round(self.gen_size * self.elitism_thresh))
        buf = self._generation[:idx]

        while (idx < self.gen_size):
            if np.random.rand() <= self.crossover_thresh:
                (parent_1, parent_2) = self._select_parents()
                children = self._crossover(parent_1, parent_2)
                for child in children:
                    if np.random.rand() <= self.mutation_thresh:
                        buf.append(child.mutate())
                    else:
                        buf.append(child)
                idx += 2
            else:
                if np.random.rand() <= self.mutation_thresh:
                    buf.append(self._generation[idx].mutate())
                else:
                    buf.append(self._generation[idx])
                idx += 1

        self._generation = list(sorted(buf, key=lambda x: x.fitness))


    def _crossover(self, chromosome_1, chromosome_2):
        """
        Mixes the two specified chromosomes, returning two new chromosomes
        that are a result of a crossover of the two original chromosomes.

        other: second chromosome to crossover

        return: two chromosomes that are crossovers between self and other
        """

        # Define a random pivot point around which the crossover will occur
        crossover_point = np.random.randint(0, NUM_GENES-1)

        # Create the new crossovered genes and chromosome
        new_chromosome_1 = chromosome_1[:crossover_point] + chromosome_2[crossover_point:]
        new_chromosome_2 = chromosome_2[:crossover_point] + chromosome_1[crossover_point:]

        return new_chromosome_1, new_chromosome_2


    def _select_parents(self):
        """
        A helper method used to select two parents from the generation using a
        tournament selection algorithm.
        """
        return (self._tournament_selection(), self._tournament_selection())


    def _tournament_selection(self):
        """
        A helper method used to select a random chromosome from the
        generation using a tournament selection algorithm.
        """
        samples = self._generation.sample(frac=0.1)
        idx = samples['fitness'].idxmin()
        return samples['chromosome'][idx]
