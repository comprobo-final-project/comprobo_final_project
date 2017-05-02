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
    Allows a robot to move to a fixed goal.
    """

    def train(self, robot):
        """
        Trains the task to find the most fit organism. Organisms are stored in a
        log file along with their fitness per generation.
        """

        log_location = 'logs/log_'+str(int(time.time()))+'.csv'
        print log_location

        GeneticAlgorithm(
            log_location=log_location,
            gen_size=100,
            num_genes=4,
            elitism_thresh=0.1,
            crossover_thresh=0.8,
            mutation_thresh=0.5,
            fitness_func=self.get_fitness_func(robot)).train()


    def visualizer_test(self, robot, organism):
        """
        Use the basic visualizer to see what the robot is doing.
        """

        simulation_visualizer = SimulationVisualizer(robot, real_world_scale=2)
        self.run_with_setup(robot, organism)


    def get_fitness_func(self, robot):
        """
        Provides a fitness function for the genetic alorigthm to optimize for.
        """

        # Using a closure here so we can hold our single robot instance
        def _fitness_func(organism):
            positions = self.run_with_setup(robot, organism)

            distances = [np.sqrt(position.x**2 + position.y**2) \
                    for position in positions] # all distances from goal
            fitness = np.mean(distances) # average distance from goal

            return round(fitness, 5)

        return _fitness_func


    def run_with_setup(self, robot, organism):
        """
        For training and testing, we want to use the same setup defined here.
        """

        robot_1.set_random_position(r=5.0)
        robot_1.set_random_direction()
        robot_2.set_random_position(r=5.0)
        robot_2.set_random_direction()

        return self._run(robots=[robot_1, robot_2], duration=20,
                organism=organism)


    def _run(self, robots, duration, organism):
        """
        Runs a robot through our function, controlled by an organism's genes.
        """

        goal_x = 0.0
        goal_y = 0.0
        positions = []

        for _ in range(int(duration * robot.resolution)):

            curr_ws = [robot.get_direction() for robot in robots]
            curr_poss = [robot.get_position() for robot in robots]

            curr_xs = [curr_pos.x for curr_pos in curr_poss]
            curr_ys = [curr_pos.y for curr_pos in curr_poss]

            positions.append(curr_poss) # store position history

            # Calculate difference between running robot's position and chasing
            # robot's position
            diff_xs = curr_xs[1] - curr_xs[0]
            diff_ys = curr_ys[1] - curr_ys[0]

            #TODO: continue porting to tag task after this point -------

            # Calculate angle to goal and distance to goal
            # http://stackoverflow.com/a/7869457/2204868
            diff_w = np.arctan2(diff_y, diff_x) - curr_w
            diff_w = (diff_w + np.pi) % (2*np.pi) - np.pi
            diff_r = np.sqrt(diff_x**2 + diff_y**2)

            # Define linear and angular velocities based on organism
            a1, b1, a2, b2 = organism
            forward_rate = a1*diff_w + b1*diff_r
            turn_rate = a2*diff_w + b2*diff_r

            # Set linear and angular velocities
            robot.set_twist(forward_rate, turn_rate)

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
    organism = [1.00000000e-02, 9.98000000e-01, 2.14310000e+01, 1.18000000e-01]

    if FLAGS.train:
        robot = SimRobot()
        task.train(robot)

    if FLAGS.visualize:
        robot = SimRobot()
        task.visualizer_test(robot, organism)

    if FLAGS.gazebo or FLAGS.real:
        robot = ModelRobot(real=FLAGS.real)
        task.run_with_setup(robot, organism)
