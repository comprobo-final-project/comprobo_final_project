import matplotlib.pyplot as plt
import numpy as np

import time

class TrainingVis():

    def __init__(self):
        self.fig = plt.figure()
        self.axes = None

        plt.ion()
        pass


    def create_fitness_per_run(self):
        self.axes = self.fig.add_subplot(111)
        self.fig.show()


    def update_fitness_per_run(self, fitness, frequency):
        self.axes.clear()
        self.axes.plot(fitness)
        self.fig.canvas.draw()
        plt.pause(1.0 / frequency)


if __name__ == "__main__":
    training_vis = TrainingVis()

    # Visuals
    training_vis.create_fitness_per_run()
    data = []
    while True:
        data.append(np.random.rand())
        training_vis.update_fitness_per_run(data, 100)
