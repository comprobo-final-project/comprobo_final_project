#!usr/bin/env python


"""
Node that subscribes to the current position of the Neato, the goal position,
and calculates a Twist message via an equation whose coefficients are
determined by the organism's genes. This calculated Twist is then published.
"""


import math
import time

# from .models.robot import Robot
from .simulator.robot import Robot
from .simulator.simulator import Simulator


class RobotController:
    """
    Dictates robot's motion based on genes. It is given a time to control
    the robot, after which it returns the robot's last position for fitness
    evaluation and then shutsdown.
    """

    def __init__(self, robot1, robot2, robot3, genes=None, simulator=None):
        """
        Initializes the node, publisher, subscriber, and the genes
        (coefficients) of the robot controller.

        genes: list of coefficients used in the function to calculate the
            robot's linear and angular velocities
        """

        self.robot1 = robot1
        self.robot2 = robot2
        self.robot3 = robot3
        self.genes = genes
        self.simulator = simulator


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
                curr_x1, curr_y1, curr_w1 = self.robot1.get_position()
                curr_x2, curr_y2, curr_w2 = self.robot2.get_position()
                curr_x3, curr_y3, curr_w3 = self.robot3.get_position()

                #TODO vvv figure this math out  vvv

                # Calculate difference between robot position and goal position
                diff_x = goal_x - curr_x
                diff_y = goal_y - curr_y

                try:
                    # Calculate angle to goal and distance to goal
                    diff_w = math.atan2(diff_y, diff_x) - curr_w
                    diff_r = math.sqrt(diff_x**2 + diff_y**2)
                except OverflowError:
                    print diff_x, diff_y

                # Define linear and angular velocities based on genes
                a1, b1, a2, b2 = self.genes
                forward_rate = a1*diff_w + b1*diff_r
                turn_rate = a2*diff_w + b2*diff_r

                # Set linear and angular velocities
                self.robot.set_twist(forward_rate, turn_rate)

                # self.simulator.update_graph()
                # time.sleep(.001)
        except KeyboardInterrupt:
            pass


        return self.robot.get_position()


if __name__ == '__main__':

    genes = [0.0, 1.0, 1.0, 0.0]
    duration = 15
    robot = Robot()
    robot.pose.position.x = 3.0
    robot.pose.position.y = 5.0
    # simulator = Simulator(robot, True)
    # simulator.render()
    # robot_controller = RobotController(robot, genes, simulator)
    robot_controller = RobotController(robot, genes)

    # Run
    robot_controller.run(duration)
