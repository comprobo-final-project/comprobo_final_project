#!usr/bin/env python

"""
Node that subscribes to the current position of the Neato, the goal position,
and calculates a Twist message via an equation whose coefficients are
determined by the organism's genes. This calculated Twist is then published.
"""

import math
import time

from .models.robot import Robot

class RobotController:
    """
    Dictates robot's motion based on genes. It is given a time to control
    the robot, after which it returns the robot's last position for fitness
    evaluation and then shutsdown.
    """

    def __init__(self, genes):
        """
        Initializes the node, publisher, subscriber, and the genes
        (coefficients) of the robot controller.

        genes: list of coefficients used in the function to calculate the
            robot's linear and angular velocities
        """
        # TODO: This is where you may want to pass in a Robot as a param.
        self.robot = Robot(self._position_listener)
        self.genes = genes

    def run(self, duration):
        """
        Main run function.
        duration : float - In seconds
        """
        end_time = time.time() + duration

        try:
            while time.time() < end_time:
                # TODO: PSEUDO-CODE:
                # Run the code for a set duration.
                # When return the value of the fitness
                pass
        except KeyboardInterrupt:
            pass

    # TODO: The logic in this method would probably be better suited for the run method
    def _position_listener(self, curr_x, curr_y):
        """
        Callback function for when the subscriber receives a new robot position.
        """
        # TODO: maybe do something besides hardcoding the goal
        # Define robot's goal end position
        goal_x = 0.0
        goal_y = 0.0

        # Calculate difference between robot position and goal position
        diff_x = goal_x - curr_x
        diff_y = goal_y - curr_y

        # Calculate angle to goal and distance to goal
        diff_w = math.atan2(diff_y, diff_x)
        diff_r = math.sqrt(diff_x**2 + diff_y**2)

        # Define linear and angular velocities based on genes
        a1, b1, c1, a2, b2, c2 = self.genes
        forward_rate = a1*diff_w + b1*diff_r + c1*diff_r**2
        turn_rate = a2*diff_w + b2*diff_r + c2*diff_r**2

        # Set linear and angular velocities
        self.robot.set_twist(forward_rate, turn_rate)


if __name__ == '__main__':
    genes = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    time_to_run = 10
    robot_controller = RobotController(genes)
    last_position = robot_controller.run(time_to_run)
