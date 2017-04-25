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

    def __init__(self, robot1, robot2, robot3, enable_render = False):
        """
        enable_render : bool - Determines whether visualizations show up
        """
        self.robot1 = robot1
        self.robot2 = robot2
        self.robot3 = robot3
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
        self.quiver_manager1 = axes.quiver(
            self.robot1.pose.position.x,
            self.robot1.pose.position.y,
            self.robot1.pose.velocity.x,
            self.robot1.pose.velocity.y,
            units = 'xy',
            angles = 'xy',
            scale_units = 'xy',
            scale = 1)

        self.quiver_manager2 = axes.quiver(
            self.robot2.pose.position.x,
            self.robot2.pose.position.y,
            self.robot2.pose.velocity.x,
            self.robot2.pose.velocity.y,
            units = 'xy',
            angles = 'xy',
            scale_units = 'xy',
            scale = 1)

        self.quiver_manager3 = axes.quiver(
            self.robot3.pose.position.x,
            self.robot3.pose.position.y,
            self.robot3.pose.velocity.x,
            self.robot3.pose.velocity.y,
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
        self.quiver_manager1.set_UVC(self.robot1.pose.velocity.x,
                self.robot1.pose.velocity.y)
        self.quiver_manager1.set_offsets((self.robot1.pose.position.x,
                self.robot1.pose.position.y))

        self.quiver_manager2.set_UVC(self.robot2.pose.velocity.x,
                self.robot2.pose.velocity.y)
        self.quiver_manager.set_offsets((self.robot2.pose.position.x,
                self.robot2.pose.position.y))

        self.quiver_manager3.set_UVC(self.robot3.pose.velocity.x,
                self.robot3.pose.velocity.y)
        self.quiver_manager.set_offsets((self.robot3.pose.position.x,
                self.robot3.pose.position.y))
        plt.draw()
        plt.pause(.001)

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
    # robot = Robot()
    # robot.pose.position.x = 0
    # robot.pose.position.y = 0
    # robot.twist.linear.x = 2
    # robot.twist.angular.z = 1
    #
    # simulator = Simulator(robot = robot, enable_render = True)
    # simulator.run()
