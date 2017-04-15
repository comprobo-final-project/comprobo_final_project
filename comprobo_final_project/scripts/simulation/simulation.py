import time

import matplotlib.pyplot as plt
import numpy as np

from robot import Robot


class Simulation:

    STEP_SIZE = 0.1
    SLEEP_DURATION_S = 0.1

    def __init__(self, enable_render = False):
        self.robot = Robot()
        self.robot.pose.position.x = 1
        self.robot.pose.position.y = 1
        self.robot.twist.linear.x = 1
        self.robot.twist.linear.y = 1

        self.enable_render = enable_render
        self.paths = None # Used for visualizations


    def render(self):
        plt.ion()

        axes = plt.gca()
        axes.set_xlim([-10, 10])
        axes.set_ylim([-10, 10])
        self.paths, = axes.plot(self.robot.pose.position.x, self.robot.pose.position.y, "o")

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('World')
        plt.grid(True)
        plt.show()


    def update_graph(self):
        self.paths.set_xdata(self.robot.pose.position.x)
        self.paths.set_ydata(self.robot.pose.position.y)
        plt.draw()
        plt.pause(0.0001)


    def run(self):
        if (self.enable_render):
            self.render()
        while True:
            if (self.enable_render):
                self.update_graph()
            self.robot.step(self.STEP_SIZE)
            time.sleep(self.SLEEP_DURATION_S)


if __name__ == "__main__":
    simulation = Simulation(enable_render = True)
    simulation.run()
