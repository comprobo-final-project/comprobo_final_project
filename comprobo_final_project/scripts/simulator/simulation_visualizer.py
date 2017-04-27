import time

import matplotlib.pyplot as plt
import numpy as np

from robot import Robot


class SimulationVisualizer:
    """
    Simulates a robot without ROS.
    """

    def __init__(self, robot, real_world_scale = 1):

<<<<<<< HEAD:comprobo_final_project/scripts/simulator/simulator.py
    def __init__(self, robot1, robot2, robot3, enable_render = False):
        """
        enable_render : bool - Determines whether visualizations show up
        """
        self.robot1 = robot1
        self.robot2 = robot2
        self.robot3 = robot3
        self.enable_render = enable_render
=======
        self.robot = robot
        self.robot.set_update_listener(self.update)
        self.real_world_scale = real_world_scale
>>>>>>> master:comprobo_final_project/scripts/simulator/simulation_visualizer.py
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


<<<<<<< HEAD:comprobo_final_project/scripts/simulator/simulator.py
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
=======
    def update(self, frequency):
        self.quiver_manager.set_UVC(self.robot.pose.velocity.x,
                self.robot.pose.velocity.y)
        self.quiver_manager.set_offsets((self.robot.pose.position.x,
                self.robot.pose.position.y))
>>>>>>> master:comprobo_final_project/scripts/simulator/simulation_visualizer.py

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        time.sleep(1.0 / (self.real_world_scale * frequency))


if __name__ == "__main__":
<<<<<<< HEAD:comprobo_final_project/scripts/simulator/simulator.py
    # robot = Robot()
    # robot.pose.position.x = 0
    # robot.pose.position.y = 0
    # robot.twist.linear.x = 2
    # robot.twist.angular.z = 1
    #
    # simulator = Simulator(robot = robot, enable_render = True)
    # simulator.run()
=======
    robot = Robot()
    robot.pose.position.x = 0
    robot.pose.position.y = 0
    robot.twist.linear.x = 3
    robot.twist.angular.z = 1
    simulation_visualizer = SimulationVisualizer(robot = robot, \
        real_world_scale = 10)

    while True:
        robot.step(2)
>>>>>>> master:comprobo_final_project/scripts/simulator/simulation_visualizer.py
