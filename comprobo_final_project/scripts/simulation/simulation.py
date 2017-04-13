import matplotlib.pyplot as plt
import numpy as np

from robot import Robot

class Simulation:

    def __init__(self):
        self.robot = Robot()
        self.robot.pose.position.x = 1
        self.robot.pose.position.y = 1


    def render(self):
        plt.scatter(self.robot.pose.position.x, self.robot.pose.position.y)

        axes = plt.gca()
        axes.set_xlim([-10, 10])
        axes.set_ylim([-10, 10])

        plt.xlabel('x label')
        plt.ylabel('y label')
        plt.title('Title')
        plt.grid(True)
        plt.show()


if __name__ == "__main__":
    simulation = Simulation()
    simulation.render()
