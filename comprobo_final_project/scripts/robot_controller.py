#!usr/bin/env python


"""
Node that subscribes to the current position of the Neato, the goal position,
and calculates a Twist message via an equation whose coefficients are
determined by the organism's genes. This calculated Twist is then published.
"""


import math
import time

from .simulator.robot import Robot


class RobotController:
    """
    Dictates robot's motion based on genes. It is given a time to control
    the robot, after which it returns the robot's last position for fitness
    evaluation and then shutsdown.
    """

    def __init__(self, robot, genes=None):
        """
        Initializes the node, publisher, subscriber, and the genes
        (coefficients) of the robot controller.

        genes: list of coefficients used in the function to calculate the
            robot's linear and angular velocities
        """

        self.robot = robot
        self.genes = genes


    def set_genes(self, genes):
        """
        sets the genes for this iteration of the robot
        """

        self.genes = genes


    def run(self, duration):
        """
        Main run function.
        duration : float - In seconds
        """

        end_time = time.time() + duration

        try:
            while time.time() < end_time:
                curr_x, curr_y, curr_w = self.robot.get_position()
                goal_x = 0.0
                goal_y = 0.0

                # Calculate difference between robot position and goal position
                diff_x = goal_x - curr_x
                diff_y = goal_y - curr_y

                try:
                    # Calculate angle to goal and distance to goal
                    goal_w = math.atan2(diff_y, diff_x)
                    diff_w = goal_w - curr_w
                    diff_r = math.sqrt(diff_x**2 + diff_y**2)
                except OverflowError:
                    print 'Overflow Error: ', diff_x, diff_y

                # Define linear and angular velocities based on genes
                a1, b1, a2, b2 = self.genes
                forward_rate = a1*diff_w + b1*diff_r
                turn_rate = a2*diff_w + b2*diff_r

                # Set linear and angular velocities
                self.robot.set_twist(forward_rate, turn_rate)

                # self.simulator.update_graph()
                # time.sleep(.1)
        except KeyboardInterrupt:
            pass

        return self.robot.poses


if __name__ == '__main__':

    from .simulator.simulation_visualizer import SimulationVisualizer

    genes = [0.0, 1.0, 1.0, 0.0]
    robot = Robot()
    robot.set_random_pose() # give the robot a random position and orientation
    robot_controller = RobotController(robot, genes)
    simulation_visualizer = SimulationVisualizer(robot, real_world_scale=10)

    # Run
    robot_controller.run(duration=15)
