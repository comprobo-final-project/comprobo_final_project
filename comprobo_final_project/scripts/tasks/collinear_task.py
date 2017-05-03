import time
import numpy as np
from ..models.robot import Robot as ModelRobot
from ..simulator.robot import Robot as SimRobot
from ..simulator.simulation_visualizer import SimulationVisualizer
from ..gene_alg_2.genetic_algorithm import GeneticAlgorithm
from ..visualizations import fitness_vs_run
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
            num_genes=12,
            elitism_thresh=0.1,
            crossover_thresh=0.8,
            mutation_thresh=0.5,
            fitness_thresh = -.95,
            fitness_func=self.get_fitness_func(robots)).train()


    def visualizer_test(self, robot, organism):
        """
        Use the basic visualizer to see what the robot is doing.
        """
        simulation_visualizer = SimulationVisualizer(robot, real_world_scale=2)
        self.run_with_setup(robot, organism)


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
            for i in range(3):

                positions = self.run_with_setup(robots, organism)
                # end = positions[-1]
                # print "ROBOT END", end[0].x, end[0].y, end[1].x, end[1].y, end[2].x, end[2].y
                r2_values = []

                for position in positions:
                    x = []
                    y = []
                    for robot in position:
                        x.append(robot.x)
                        y.append(robot.y)

                    _, _, r_value, _, _ = stats.linregress(zip(x,y))
                    r2_values.append(r_value**2)

                final_value = np.mean(r2_values)
                fitness.append(final_value)

            overall_fitness = np.mean(fitness)

            # currently negating to match with the generation's determination of best fitness
            # print -1*overall_fitness
            return -1*overall_fitness

        return _get_fitness_collinear


    def run_with_setup(self, robots, organism):
        """
        For training and testing, we want to use the same setup defined here.
        """
        for robot in robots:
            robot.set_random_position(r=5.0)
            robot.set_random_direction()
        return self._run(robots=robots, duration=20, organism=organism)


    def _run(self, robots, duration, organism):
        """
        Runs a robot through our function, controlled by an organism's genes.
        """
        robot_positions = []
        for _ in range(int(duration * robots[0].resolution)):

            positions = []
            directions = []
            for robot in robots:
                positions.append(robot.get_position())
                directions.append(robot.get_direction())

            # Calculate difference between robot position and other robots position
            diff_x21 = positions[1].x - positions[0].x
            diff_y21 = positions[1].y - positions[0].y

            diff_x31 = positions[2].x - positions[0].x
            diff_y31 = positions[2].y - positions[0].y

            diff_x32 = positions[2].x - positions[1].x
            diff_y32 = positions[2].y - positions[1].y

            try:
                # Calculate angle to goal and distance to goal
                diff_w21 = np.arctan2(diff_y21, diff_x21) - directions[0]
                diff_w21 = (diff_w21 + np.pi) % (2*np.pi) - np.pi
                diff_w12 = np.arctan2(diff_y21, diff_x21) - directions[1]
                diff_w12 = (diff_w12 + np.pi) % (2*np.pi) - np.pi
                diff_r21 = np.sqrt(diff_x21**2 + diff_y21**2)

                diff_w31 = np.arctan2(diff_y31, diff_x31) - directions[0]
                diff_w31 = (diff_w31 + np.pi) % (2*np.pi) - np.pi
                diff_w13 = np.arctan2(diff_y31, diff_x31) - directions[2]
                diff_w13 = (diff_w13 + np.pi) % (2*np.pi) - np.pi
                diff_r31 = np.sqrt(diff_x31**2 + diff_y31**2)

                diff_w32 = np.arctan2(diff_y32, diff_x32) - directions[1]
                diff_w32 = (diff_w32 + np.pi) % (2*np.pi) - np.pi
                diff_w23 = np.arctan2(diff_y32, diff_x32) - directions[2]
                diff_w23 = (diff_w23 + np.pi) % (2*np.pi) - np.pi
                diff_r32 = np.sqrt(diff_x32**2 + diff_y32**2)
            except OverflowError:
                print diff_x21, diff_y21, diff_x31, diff_y31, diff_x32, diff_y32

            # Define linear and angular velocities based on genes
            a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3 = organism

            # calculate movements for each robot
            forward_rate1 = a2*diff_w21 + b2*diff_r21 + a3*diff_w31 + b3*diff_r31
            turn_rate1 = c2*diff_w21 + d2*diff_r21 + c3*diff_w31 + d3*diff_r31

            forward_rate2 = a1*diff_w12 + b1*diff_r21 + a3*diff_w32 + b3*diff_r32
            turn_rate2 = c1*diff_w12 + d1*diff_r21 + c3*diff_w32 + d3*diff_r32

            forward_rate3 = a1*diff_w13 + b1*diff_r31 + a2*diff_w23 + b2*diff_r32
            turn_rate3 = c1*diff_w13 + d1*diff_r31 + c2*diff_w23 + d2*diff_r32

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

    #TODO: need an organism

    task = CollinearTask()
    sim_robots = [SimRobot() for i in range(3)]
    model_robots = [ModelRobot(real=FLAGS.real, name="robot"+str(i)) for i in range(1,4)]


    if FLAGS.train:
        task.train(sim_robots)

    if FLAGS.visualize:
        task.visualizer_test(sim_robots, organism)

    if FLAGS.gazebo or FLAGS.real:
        task.run_with_setup(model_robots, organism)
