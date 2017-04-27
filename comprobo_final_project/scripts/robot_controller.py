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

    def __init__(self, robots, genes=None):

        """
        Initializes the node, publisher, subscriber, and the genes
        (coefficients) of the robot controller.

        genes: list of coefficients used in the function to calculate the
            robot's linear and angular velocities
        """

        self.robots = robots
        self.genes = genes


    def set_genes(self, genes):
        """
        sets the genes for this iteration of the robot
        """

        self.genes = genes


    def calculate_twists_collinear(self, positions):
        """
        calculates twists for the collinear task
        """

        # Calculate difference between robot position and other robots position
        diff_x21 = positions[1][0] - positions[0][0]
        diff_y21 = positions[1][1] - positions[0][1]

        diff_x31 = positions[2][0] - positions[0][0]
        diff_y31 = positions[2][1] - positions[0][1]

        diff_x32 = positions[2][0] - positions[1][0]
        diff_y32 = positions[2][1] - positions[1][1]

        try:
            # Calculate angle to goal and distance to goal
            diff_w21 = math.atan2(diff_y21, diff_x21) - positions[0][2]
            diff_w12 = math.atan2(diff_y21, diff_x21) - positions[1][2]
            diff_r21 = math.sqrt(diff_x21**2 + diff_y21**2)

            diff_w31 = math.atan2(diff_y31, diff_x31) - positions[0][2]
            diff_w13 = math.atan2(diff_y31, diff_x31) - positions[2][2]
            diff_r31 = math.sqrt(diff_x31**2 + diff_y31**2)

            diff_w32 = math.atan2(diff_y32, diff_x32) - positions[1][2]
            diff_w23 = math.atan2(diff_y32, diff_x32) - positions[2][2]
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

        twists = [(forward_rate1, turn_rate1), (forward_rate2, turn_rate2), (forward_rate3, turn_rate3)]

        # Set linear and angular velocities
        index =  0
        for robot in self.robots:
            robot.set_twist(twists[index][0], twists[index][1])
            index += 1


    def calculate_twists_simple(self, positions):
        """
        calculates twist messages for the simple task
        """

        goal_x = 0.0
        goal_y = 0.0

        # Calculate difference between robot position and goal position
        diff_x = goal_x - positions[0][0]
        diff_y = goal_y - positions[0][1]

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

        twists = [(forward_rate, turn_rate)]

        # Set linear and angular velocities
        index = 0
        for robot in self.robots:
            robot.set_twist(twists[index][0], twists[index][1])
            index += 1


    def run(self, duration):
        """
        Main run function.
        duration : float - In seconds
        """

        end_time = time.time() + duration

        try:
            while time.time() < end_time:
                positions = []
                for robot in self.robots:
                    positions.append(robot.get_position())

                    self.calculate_twists_collinear(positions)

        except KeyboardInterrupt:
            pass

        end_positions = []
        for robot in self.robots:
            end_positions.append(robot.get_position())

        return end_positions


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
