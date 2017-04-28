class GeneticAlgorithm(object):

    def __init__(self, size=1024, crossover=0.8, elitism=0.1, mutation=0.05, \
        num_genes=4, fitness_function):

        self.elitism = elitism
        self.mutation = mutation
        self.crossover = crossover
        self.fitness_function = fitness_function
        self.tournament_size = int(size * 0.005)

        population = self.generate_population(size, num_genes)

    def generate_population(self, size, num_genes):
        population = []

        for _ in range(size):
            chromosome = []
            for _ in range(num_genes):
                chromosome.append(round(random.uniform(-5, 5), 3))
            population.append(chromosome)

        return population

    def calculate_fitness(self, population):
        # TODO: Get fitness array of population.

        fitness_function()
