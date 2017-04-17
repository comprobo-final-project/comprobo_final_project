import time

import matplotlib.pyplot as plt
import numpy as np

from robot import Robot


class Simulator:
    """
    Simulates a robot without ROS.
    """

    STEP_SIZE = 0.1
    SLEEP_DURATION_S = 0.01

    def __init__(self, robot, enable_render = False):
        """
        enable_render : bool - Determines whether visualizations show up
        """
        self.robot = robot
        self.enable_render = enable_render
        self.quiver_manager = None # Used for visualizations


    def render(self):
        """
        Renders the simulated world
        """
        plt.ion()

        axes = plt.gca()
        axes.set_xlim([-10, 10])
        axes.set_ylim([-10, 10])

        # http://matplotlib.org/api/pyplot_api.html#matplotlib.pyplot.quiver
        self.quiver_manager = axes.quiver(
            self.robot.pose.position.x,
            self.robot.pose.position.y,
            self.robot.pose.velocity.x,
            self.robot.pose.velocity.y,
            units = 'xy',
            angles = 'xy',
            scale_units = 'xy',
            scale = 1)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('World')
        plt.grid(True)
        plt.show()


    def update_graph(self):
        self.quiver_manager.set_UVC(self.robot.pose.velocity.x, self.robot.pose.velocity.y)
        self.quiver_manager.set_offsets((self.robot.pose.position.x, self.robot.pose.position.y))
        plt.draw()


    def run(self):
        if (self.enable_render):
            self.render()
        while True:
            self.robot.step(self.STEP_SIZE)
            if (self.enable_render):
                self.update_graph()
                plt.pause(self.SLEEP_DURATION_S)
            else:
                time.sleep(self.SLEEP_DURATION_S)


if __name__ == "__main__":
    robot = Robot()
    robot.pose.position.x = 0
    robot.pose.position.y = 0
    robot.twist.linear.x = 2
    robot.twist.angular.z = 1

    simulator = Simulator(robot = robot, enable_render = True)
    simulator.run()
