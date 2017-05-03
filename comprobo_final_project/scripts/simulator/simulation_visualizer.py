from ..helpers import sleeper

import matplotlib.pyplot as plt
import numpy as np

from robot import Robot


class SimulationVisualizer:
    """
    Simulates a robot without ROS.
    """

    def __init__(self, robots, real_world_scale = 1):
        """
        enable_render : bool - Determines whether visualizations show up
        """
        self.robots = robots

        for robot in self.robots:
            robot.set_update_listener(self.update)
        self.real_world_scale = real_world_scale
        self.quiver_manager = None # Used for visualizations

        # We use this value to debounce update calls
        self.update_count = 0

        self.render()


    def render(self):
        """
        Renders the simulated world
        """
        plt.ion()

        self.fig, axes = plt.subplots()
        axes = plt.gca()
        axes.set_xlim([-10, 10])
        axes.set_ylim([-10, 10])

        # http://matplotlib.org/api/pyplot_api.html#matplotlib.pyplot.quiver

        # currently will show only one robot, because I'm not sure how this works for multiple
        self.quiver_manager = axes.quiver(
            [robot.pose.position.x for robot in self.robots],
            [robot.pose.position.y for robot in self.robots],
            [robot.pose.velocity.x for robot in self.robots],
            [robot.pose.velocity.y for robot in self.robots],
            units = 'xy',
            angles = 'xy',
            scale_units = 'xy',
            scale = 1)

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('World')
        plt.grid(True)
        plt.show()

    def update(self, frequency):
        if self.update_count % len(self.robots) == 0:
            self.update_count = 0

            self.quiver_manager.set_UVC(
                [robot.pose.velocity.x for robot in self.robots],
                [robot.pose.velocity.y for robot in self.robots])
            self.quiver_manager.set_offsets((
                [robot.pose.position.x for robot in self.robots],
                [robot.pose.position.y for robot in self.robots]))
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            sleeper.sleep(1.0 / (self.real_world_scale * frequency))

        self.update_count += 1


if __name__ == "__main__":

    robot = Robot()
    robot.pose.position.x = 0
    robot.pose.position.y = 0
    robot.twist.linear.x = 3
    robot.twist.angular.z = 1
    simulation_visualizer = SimulationVisualizer(robot = robot, \
        real_world_scale = 10)

    while True:
        robot.step(2)
