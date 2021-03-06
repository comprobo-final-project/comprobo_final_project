"""
An Attempted Task class that would evolve organisms to revolve around a point
"""

import time
import numpy as np
from ..gene_alg_2.genetic_algorithm import GeneticAlgorithm


class OrbitalTask(object):
    """
    Allows a robot to orbit around a fixed goal.
    """

    def train(self, robot):
        """
        Trains the task to find the most fit organism. Organisms are stored in a
        log file along with their fitness per generation.
        """

        log_location = 'logs/orbital_'+str(int(time.time()))+'.csv'
        print log_location

        GeneticAlgorithm(
            log_location=log_location,
            gen_size=100,
            num_organisms=1,
            num_genes=8,
            fitness_thresh=0.05,
            elitism_thresh=0.1,
            crossover_thresh=0.8,
            mutation_thresh=0.1,
            fitness_func=self.get_fitness_func(robot)).train()


    def visualizer_test(self, robot, organisms):
        """
        Use the basic visualizer to see what the robot is doing.
        """

        simulation_visualizer = SimulationVisualizer([robot], real_world_scale=2)
        get_fitness = self.get_fitness_func(robot)
        fitness = get_fitness(organisms)
        print fitness


    def get_fitness_func(self, robot):
        """
        Provides a fitness function for the genetic alorigthm to optimize for.
        """

        # Using a closure here so we can hold our single robot instance
        def _fitness_func(organisms):

            avg_fit = []

            for _ in range(3):
                positions = self.run_with_setup(robot, organisms[0])

                ideal_radius = 1.5 # Let it rotate 1m away from goal

                radius_err = np.mean([np.absolute(
                    np.sqrt(position[0]**2 + position[1]**2) - ideal_radius) \
                    for position in positions])

                centroid = positions.mean(axis=0)
                centroid_err = np.sqrt(centroid[0]**2 + centroid[1]**2)

                fitness = radius_err * centroid_err + radius_err + centroid_err
                avg_fit.append(fitness)

            return np.mean(avg_fit)

        return _fitness_func


    def run_with_setup(self, robot, organism, duration=20):
        """
        For training and testing, we want to use the same setup defined here.
        """

        robot.set_random_position(r=1.5)
        robot.set_random_direction()
        return self._run(robot=robot, duration=duration, organism=organism)


    def _run(self, robot, duration, organism):
        """
        Runs a robot through our function, controlled by an organism's genes.
        """

        goal_x = 0.0
        goal_y = 0.0
        positions = []

        for _ in range(int(duration * robot.resolution)):

            curr_w = robot.get_direction()
            curr_pos = robot.get_position()

            curr_x = curr_pos.x
            curr_y = curr_pos.y

            positions.append((curr_pos.x, curr_pos.y)) # store position history

            # Calculate difference between robot position and goal position
            diff_x = goal_x - curr_x
            diff_y = goal_y - curr_y

            # Calculate angle to goal and distance to goal
            # http://stackoverflow.com/a/7869457/2204868
            diff_w = np.arctan2(diff_y, diff_x) - curr_w
            diff_w = (diff_w + np.pi) % (2*np.pi) - np.pi
            diff_r = np.sqrt(diff_x**2 + diff_y**2)

            # Define linear and angular velocities based on organism
            a1, b1, a2, b2, c1, c2, d1, d2 = organism
            forward_rate = a1*diff_w + b1*diff_r + c1*diff_w**2 + d1*diff_r**2
            turn_rate = a2*diff_w + b2*diff_r + + c2*diff_w**2 + d2*diff_r**2

            # Set linear and angular velocities
            robot.set_twist(forward_rate, turn_rate)

        return np.asarray(positions)


if __name__ == "__main__":

    import argparse

    # All tasks should support a standard set of commands similar to this
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', action='store_true')
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--gazebo', action='store_true')
    parser.add_argument('--real', action='store_true')

    FLAGS, _ = parser.parse_known_args()

    # Initialize task
    task = OrbitalTask()

    organisms = [[141.013, 1000.0, 0.922, 0.457, 1000.0, 0.327, 1000.0, 0.129]]

    # Create robots, both simulation ones and real ones
    from ..simulator.robot import Robot as SimRobot
    sim_robot = SimRobot(noise=0.1)

    if FLAGS.train:
        task.train(sim_robot)

    if FLAGS.visualize:
        from ..simulator.simulation_visualizer import SimulationVisualizer
        task.visualizer_test(sim_robot, organisms)

    if FLAGS.real or FLAGS.gazebo:
        from ..models.robot import Robot as ModelRobot
        model_robot = ModelRobot(real=FLAGS.real, name="robot1")
        task.run_with_setup(model_robot, organisms[0], duration=300)
