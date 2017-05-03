import time
import numpy as np

from ..gene_alg_2.genetic_algorithm import GeneticAlgorithm
from scipy import stats


class CollinearTask(object):
    """
    Allows a robot to move to a fixed goal.
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
            gen_size=100,
            num_genes=24,
            elitism_thresh=0.1,
            crossover_thresh=0.8,
            mutation_thresh=0.2,
            fitness_thresh = -.95,
            fitness_func=self.get_fitness_func(robots)).train()


    def visualizer_test(self, robots, organism):
        """
        Use the basic visualizer to see what the robots is doing.
        """
        from ..simulator.simulation_visualizer import SimulationVisualizer
        simulation_visualizer = SimulationVisualizer(robots, real_world_scale=10)
        get_fitness = self.get_fitness_func(robots)
        print get_fitness(organism)
        # self.run_with_setup(robots, organism)


    def get_fitness_func(self, robots):
        """
        Provides a fitness function for the genetic alorigthm to optimize for.
        """

        # Using a closure here so we can hold our single robot instance
        def _get_fitness_collinear(organism):
            """
            Calculate the fitness of a specified organism for collinear task
            """
            fitness = []

            for i in range(4):
                positions = self.run_with_setup(robots, organism)
                r2_values = []

                for position in positions:
                    robots_xy = [(robot.x, robot.y) for robot in position]
                    _, _, r_value, _, _ = stats.linregress(robots_xy)
                    r2_values.append(r_value**2)

                final_value = np.mean(r2_values)
                fitness.append(final_value)

            worst_fit = np.min(fitness)

            # currently negating to match with the generation's determination of best fitness
            return -1*worst_fit

        return _get_fitness_collinear


    def run_with_setup(self, robots, organism):
        """
        For training and testing, we want to use the same setup defined here.
        """
        for robot in robots:
            robot.set_random_position(r=5.0)
            robot.set_random_direction()
        return self._run(robots=robots, duration=15, organism=organism)


    def _run(self, robots, duration, organism):
        """
        Runs a robot through our function, controlled by an organism's genes.
        """
        robot_positions = []
        for _ in range(int(duration * robots[0].resolution)):

            positions = [robot.get_position() for robot in robots]
            directions = [robot.get_direction() for robot in robots]

            # Calculate difference between robot position and other robots position
            diff_10 = positions[1] - positions[0]
            diff_20 = positions[2] - positions[0]
            diff_21 = positions[2] - positions[1]

            angle_10 = np.arctan2(diff_10.y, diff_10.x)
            angle_20 = np.arctan2(diff_20.y, diff_20.x)
            angle_21 = np.arctan2(diff_21.y, diff_21.x)

            try:
                # Calculate angle to goal and distance to goal
                diff_w10 = angle_10 - directions[0]
                diff_w10 = (diff_w10 + np.pi) % (2*np.pi) - np.pi
                diff_w01 = angle_10 - directions[1]
                diff_w01 = (diff_w10 + np.pi) % (2*np.pi) - np.pi
                diff_r10 = np.sqrt(diff_10.x**2 + diff_10.y**2)

                diff_w20 = angle_20 - directions[0]
                diff_w20 = (diff_w20 + np.pi) % (2*np.pi) - np.pi
                diff_w02 = angle_20 - directions[2]
                diff_w02 = (diff_w02 + np.pi) % (2*np.pi) - np.pi
                diff_r20 = np.sqrt(diff_20.x**2 + diff_20.y**2)

                diff_w21 = angle_21 - directions[1]
                diff_w21 = (diff_w21 + np.pi) % (2*np.pi) - np.pi
                diff_w12 = angle_21 - directions[2]
                diff_w12 = (diff_w12 + np.pi) % (2*np.pi) - np.pi
                diff_r21 = np.sqrt(diff_21.x**2 + diff_21.y**2)

            except OverflowError:
                print diff_10.x, diff_10.y, diff_20.x, diff_20.y, diff_21.x, diff_21.y

            # Define linear and angular velocities based on genes
            a1, a2, a3, a4, b1, b2, b3, b4, c1, c2, c3, c4, d1, d2, d3, d4, \
                e1, e2, e3, e4, f1, f2, f3, f4 = organism

            # calculate movements for each robot
            forward_rate1 = a1*diff_w10 + a2*diff_r10 + a3*diff_w20 + a4*diff_r20
            turn_rate1 = b1*diff_w10 + b2*diff_r10 + b3*diff_w20 + b4s*diff_r20

            forward_rate2 = c1*diff_w01 + c2*diff_r10 + c3*diff_w21 + c4*diff_r21
            turn_rate2 = d1*diff_w01 + d2*diff_r10 + d3*diff_w21 + d4*diff_r21

            forward_rate3 = e1*diff_w02 + e2*diff_r20 + e3*diff_w12 + e4*diff_r21
            turn_rate3 = f1*diff_w02 + f2*diff_r20 + f3*diff_w12 + f4*diff_r21

            twists = [(forward_rate1, turn_rate1), (forward_rate2, turn_rate2), (forward_rate3, turn_rate3)]

            # Set linear and angular velocities
            for idx, robot in enumerate(robots):
                robot.set_twist(twists[idx][0], twists[idx][1])

            robot_positions.append(positions)

        return robot_positions


if __name__ == "__main__":

    import argparse

    # All tasks should support a standard set of commands similar to this
    parser = argparse.ArgumentParser()
    parser.add_argument('--train', action='store_true')
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--gazebo', action='store_true')
    parser.add_argument('--real', action='store_true')

    FLAGS, _ = parser.parse_known_args()

    task = CollinearTask()

    from ..simulator.robot import Robot as SimRobot
    sim_robots = [SimRobot() for i in range(3)]

    organism = [7.17100000e+00, 1.87000000e-01, 3.96000000e-01, 3.03600000e+00, 9.00000000e-03, 7.24700000e+00, 1.36850000e+01, 1.47000000e-01 , 7.49000000e-01, 2.91200000e+00, 3.63000000e-01, 2.00000000e-02]

    if FLAGS.train:
        task.train(sim_robots)

    if FLAGS.visualize:
        task.visualizer_test(sim_robots, organism)

    if FLAGS.gazebo or FLAGS.real:
        from ..models.robot import Robot as ModelRobot
        model_robots = [ModelRobot(real=FLAGS.real) for i in range(3)]
        task.run_with_setup(model_robots, organism)
