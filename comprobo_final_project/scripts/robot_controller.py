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

    def __init__(self, robot1, robot2, robot3, genes=None):

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

                # Calculate difference between robot position and other robots position
                diff_x21 = curr_x2 - curr_x1
                diff_y21 = curr_y2 - curr_y1

                diff_x31 = curr_x3 - curr_x1
                diff_y31 = curr_y3 - curr_y1

                diff_x32 = curr_x3 - curr_x2
                diff_y32 = curr_y3 - curr_y2

                try:
                    # Calculate angle to goal and distance to goal
                    diff_w21 = math.atan2(diff_y21, diff_x21) - curr_w1
                    diff_w12 = math.atan2(diff_y21, diff_x21) - curr_w2
                    diff_r21 = math.sqrt(diff_x21**2 + diff_y21**2)

                    diff_w31 = math.atan2(diff_y31, diff_x31) - curr_w1
                    diff_w13 = math.atan2(diff_y31, diff_x31) - curr_w3
                    diff_r31 = math.sqrt(diff_x31**2 + diff_y31**2)

                    diff_w32 = math.atan2(diff_y32, diff_x32) - curr_w2
                    diff_w23 = math.atan2(diff_y32, diff_x32) - curr_w3
                    diff_r32 = math.sqrt(diff_x32**2 + diff_y32**2)
                except OverflowError:
                    print diff_x21, diff_y21, diff_x31, diff_y31, diff_x32, diff_y32

                # Define linear and angular velocities based on genes
                a1, b1, c1, d1, a2, b2, c2, d2, a3, b3, c3, d3 = self.genes

                # calculate movements for each robot
                forward_rate1 = a2*diff_w21 + b2*diff_r21 + a3*diff_w31 + b3*diff_r31
                turn_rate1 = c2*diff_w21 + d2*diff_r21 + c3*diff_w31 + d3*diff_r31

                forward_rate2 = a1*diff_w12 + b1*diff_r21 + a3*diff_w32 + b3*diff_r32
                turn_rate2 = c1*diff_w12 + d1*diff_r21 + c3*diff_w32 + d3*diff_r32

                forward_rate3 = a1*diff_w13 + b1*diff_r31 + a2*diff_w23 + b2*diff_r32
                turn_rate3 = c1*diff_w13 + d1*diff_r31 + c2*diff_w23 + d2*diff_r32

                # Set linear and angular velocities
                self.robot1.set_twist(forward_rate1, turn_rate1)
                self.robot2.set_twist(forward_rate2, turn_rate2)
                self.robot3.set_twist(forward_rate3, turn_rate3)

        except KeyboardInterrupt:
            pass


        return self.robot.get_position()


if __name__ == '__main__':

    from .simulator.simulation_visualizer import SimulationVisualizer

    genes = [0.0, 1.0, 1.0, 0.0]

    robot = Robot()
    robot.pose.position.x = 3.0
    robot.pose.position.y = 5.0


    robot_controller = RobotController(robot, genes)
    simulation_visualizer = SimulationVisualizer(robot, real_world_scale = 10)

    # Run
    robot_controller.run(duration = 15)
