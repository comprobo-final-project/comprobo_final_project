import numpy as np

from ..simulator.robot import Robot
from ..gene_alg_2.genetic_algorithm import GeneticAlgorithm


class GoalTask(object):


    def __init__(self):
        self.robot = Robot(noise=0.0)


    def reset_robot(self):
        self.robot.set_random_position(r=5.0)
        self.robot.set_random_direction()


    def run(self, duration, genes):

        goal_x = 0.0
        goal_y = 0.0
        positions = []

        for _ in range(int(duration * self.robot.resolution)):

            curr_w = self.robot.get_direction()
            curr_pos = self.robot.get_position()

            curr_x = curr_pos.x
            curr_y = curr_pos.y

            positions.append(curr_pos) # store position history

            # Calculate difference between robot position and goal position
            diff_x = goal_x - curr_x
            diff_y = goal_y - curr_y

            # Calculate angle to goal and distance to goal
            # http://stackoverflow.com/a/7869457/2204868
            diff_w = np.arctan2(diff_y, diff_x) - curr_w
            diff_w = (diff_w + np.pi) % (2*np.pi) - np.pi
            diff_r = np.sqrt(diff_x**2 + diff_y**2)

            # Define linear and angular velocities based on genes
            a1, b1, a2, b2 = genes
            forward_rate = a1*diff_w + b1*diff_r
            turn_rate = a2*diff_w + b2*diff_r

            # Set linear and angular velocities
            self.robot.set_twist(forward_rate, turn_rate)

        return positions


    def train(self):
        GeneticAlgorithm(
            gen_size=100,
            num_genes=4,
            elitism_thresh=0.1,
            crossover_thresh=0.8,
            mutation_thresh=0.5,
            fitness_func=self._fitness_function).train()


    def test(self):
        pass


    def _fitness_function(self, chromosome):
        self.reset_robot()
        positions = self.run(duration=20, genes=chromosome)

        distances = [np.sqrt(position.x**2 + position.y**2) \
                for position in positions] # all distances from goal
        fitness = np.mean(distances) # average distance from goal

        return fitness


if __name__ == "__main__":
    task = GoalTask()
    task.train()
