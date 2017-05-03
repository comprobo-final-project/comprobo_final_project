from ..helpers import sleeper

import matplotlib.pyplot as plt
import numpy as np

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

            self.quiver_manager.set_offsets(
                [(robot.pose.position.x, robot.pose.position.y) \
                    for robot in self.robots])
            self.quiver_manager.set_UVC(
                [robot.pose.velocity.x for robot in self.robots],
                [robot.pose.velocity.y for robot in self.robots])
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            sleeper.sleep(1.0 / (self.real_world_scale * frequency))

        self.update_count += 1


if __name__ == "__main__":
    from robot import Robot

    rb1 = Robot(noise = 0, resolution=4)
    rb1.pose.position.x = 0
    rb1.pose.position.y = 0
    rb2 = Robot(noise = 0, resolution=4)
    rb2.pose.position.x = 0
    rb2.pose.position.y = 0
    simulation_visualizer = SimulationVisualizer(robots=[rb1, rb2], \
        real_world_scale = 3)

    while True:
        rb1.set_twist(1, 0.3)
        rb2.set_twist(1, -0.3)
