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
            [self.robots[0].pose.position.x, self.robots[1].pose.position.x, self.robots[2].pose.position.x],
            [self.robots[0].pose.position.y, self.robots[1].pose.position.y, self.robots[2].pose.position.y],
            [self.robots[0].pose.velocity.x, self.robots[1].pose.velocity.x, self.robots[2].pose.velocity.x],
            [self.robots[0].pose.velocity.y, self.robots[1].pose.velocity.y, self.robots[2].pose.velocity.y],
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
        self.quiver_manager.set_UVC([self.robots[0].pose.velocity.x, self.robots[1].pose.velocity.x, self.robots[2].pose.velocity.x],
                                    [self.robots[0].pose.velocity.y, self.robots[1].pose.velocity.y, self.robots[2].pose.velocity.y])
        self.quiver_manager.set_offsets(([self.robots[0].pose.position.x, self.robots[1].pose.position.x, self.robots[2].pose.position.x],
                                        [self.robots[0].pose.position.y, self.robots[1].pose.position.y, self.robots[2].pose.position.y]))

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        sleeper.sleep(1.0 / (self.real_world_scale * frequency))


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
