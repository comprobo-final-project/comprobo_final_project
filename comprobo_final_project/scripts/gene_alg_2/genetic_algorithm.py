import csv
import numpy as np
from .generation import Generation


class GeneticAlgorithm(object):

    def __init__(
        self,
        fitness_func,
        log_location,
        gen_size=1024,
        num_genes=4,
        train_thresh=10000,
        fitness_thresh=0.25,
        elitism_thresh=0.1,
        crossover_thresh=0.8,
        mutation_thresh=0.05):

        self.log_location = log_location

        # Used for training
        self.train_thresh = train_thresh
        self.fitness_thresh = fitness_thresh

        # Rest of the hyperparams are used for evolution
        self._generation = Generation(
            gen_size=gen_size,
            num_genes=num_genes,
            elitism_thresh=elitism_thresh,
            crossover_thresh=crossover_thresh,
            mutation_thresh=mutation_thresh,
            fitness_func=fitness_func)


    def train(self):

        print "Starting the training session."

        with open(self.log_location,'wb') as file_obj:

            writer = csv.writer(file_obj, delimiter = ',')
            gen_idx = 0
            found = False

            while gen_idx < self.train_thresh:

                # Evaluate the fitness for one generation
                self._generation.evaluate_fitness()
                organism, fitness = self._generation.get_zeroth()

                # Print out the best
                print"Generation %d: %s" % (gen_idx, organism), fitness

                # Save to the log
                writer.writerow([gen_idx, organism, fitness])
                file_obj.flush()

                if fitness < self.fitness_thresh:
                    print "Most fit gene:", organism, fitness
                    found = True
                    break
                else:
                    self._generation.evolve()

                gen_idx += 1

        if not found:
            print "Maximum generations reached without success."
