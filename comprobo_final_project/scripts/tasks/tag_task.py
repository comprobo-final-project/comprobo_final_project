#!usr/bin/env python


import time
import numpy as np
from ..models.robot import Robot as ModelRobot
from ..simulator.robot import Robot as SimRobot
from ..simulator.simulation_visualizer import SimulationVisualizer
from ..gene_alg_2.genetic_algorithm import GeneticAlgorithm
from ..visualizations import fitness_vs_run


class TagTask(object):
    """
    Allow two robots to play tag. One robot chases and one robot runs away.
    """

    def train(self, robots):
        """
        Trains the task to find the most fit organism. Organisms are stored in a
        log file along with their fitness per generation.
        """

        log_location = 'logs/log_'+str(int(time.time()))+'.csv'
        print log_location

        GeneticAlgorithm(
            log_location=log_location,
            gen_size=10,
            num_genes=4,
            num_organisms=2,
            elitism_thresh=0.1,
            crossover_thresh=0.8,
            mutation_thresh=0.5,
            fitness_func=self.get_fitness_func(robots)).train()


    def visualizer_test(self, robots, organism):
        """
        Use the basic visualizer to see what the robot is doing.
        """

        simulation_visualizer = SimulationVisualizer(robots, real_world_scale=2)
        self.run_with_setup(robots, organism)


    def get_fitness_func(self, robots):
        """
        Provides a fitness function for the genetic alorigthm to optimize for.
        """

        chasing_robot, running_robot = robots 

        # Using a closure here so we can hold our single robot instance
        def _fitness_func(organism):
            positions = self.run_with_setup(robots, organism)

            distances = [np.sqrt(position.x**2 + position.y**2) \
                    for position in positions] # all distances from goal
            fitness = round(np.mean(distances), 5) # average distance from goal

            return fitness, fitness * -1

        return _fitness_func


    def run_with_setup(self, robots, organisms):
        """
        For training and testing, we want to use the same setup defined here.
        """

        # Give every robot in robots a random pose
        for robot in robots:
            robot.set_random_position(r=3.0)
            robot.set_random_direction()

        return self._run(robots=robots, duration=20, organisms=organisms)


    def _run(self, robots, duration, organisms):
        """
        Runs a robot through our function, controlled by an organism's genes.

        organisms[unique organism number][gene index]
        """

        goal_x = 0.0
        goal_y = 0.0
        positions = []

        for _ in range(int(duration * robots[0].resolution)):

            curr_ws = [robot.get_direction() for robot in robots]
            curr_poss = [robot.get_position() for robot in robots]

            curr_xs = [curr_pos.x for curr_pos in curr_poss]
            curr_ys = [curr_pos.y for curr_pos in curr_poss]

            positions.append(curr_poss) # store position history

            # Calculate difference between two robots' position components
            diff_x = curr_xs[1] - curr_xs[0]
            diff_y = curr_ys[1] - curr_ys[0]

            # Calculate the amount each robot would have to turn in order
            # to be facing the other robot
            # http://stackoverflow.com/a/7869457/2204868
            diff_ws = [((np.arctan2(diff_y, diff_x) - curr_w) + np.pi) % \
                    (2*np.pi) - np.pi for curr_w in curr_ws]

            # Calculate distance between the two robots
            diff_r = np.sqrt(diff_x**2 + diff_y**2)

            # Define linear and angular velocities based on organism
            forward_rate = np.array([])
            turn_rate = np.array([])
            print "Organisms: ", organisms
            for organism_num, organism_gene in enumerate(organisms):
                print "Organism genes: ", organism_gene
                forward_rate = np.append(forward_rate, organism_gene[0] * \
                        diff_w + organism_gene[1] * diff_r)
                turn_rate = np.append(turn_rate, organism_gene[2] * diff_w + \
                        organism_gene[3] * diff_r)

            # Set linear and angular velocities
            for robot_num, robot in enumerate(robots):
                robot.set_twist(forward_rate[robot_num], turn_rate[robot_num])

        return positions


if __name__ == "__main__":

    import argparse

    # All tasks should support a standard set of commands similar to this
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', action='store_true')
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--gazebo', action='store_true')
    parser.add_argument('--real', action='store_true')

    FLAGS, _ = parser.parse_known_args()

    task = TagTask()
    # organism = [13.909, 3.869, 40.304, 0.184] # Temporary
    organisms = [[1.000e-02, 9.980e-01, 2.143e+01, 1.180e-01],
            [1.000e-02, 9.980e-01, 2.143e+01, 1.180e-01]]

    if FLAGS.train:
        chasing_robot = SimRobot()
        running_robot = SimRobot()
        task.train([chasing_robot, running_robot])

    if FLAGS.visualize:
        chasing_robot = SimRobot()
        running_robot = SimRobot()
        task.visualizer_test([chasing_robot, running_robot], organisms)

    if FLAGS.gazebo or FLAGS.real:
        chasing_robot = ModelRobot(real=FLAGS.real)
        running_robot = ModelRobot(real=FLAGS.real)
        task.run_with_setup([chasing_robot, running_robot], organisms)
