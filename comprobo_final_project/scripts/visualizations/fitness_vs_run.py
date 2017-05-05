"""
Visualization script that takes logs and plots the fitness of generations in
a given simulation
"""

import time
import seaborn
import json

import pandas as pd
import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

seaborn.set()

def graph(log_location):

    df = pd.read_csv(log_location, header=None)
    df[2] = df[2].apply(_to_list)
    b, a = signal.butter(3, 0.03)

    # Plot all fitnesses
    for i in range(len(df[2][0])):
        y_data = df[2].apply(lambda x: _get_inner_column(x, i))
        plt.plot(y_data, alpha=0.3)
        y = signal.filtfilt(b, a, y_data)
        plt.plot(y)

    plt.xlabel('Generations')
    plt.ylabel('Fitness')

    plt.show()


def _to_list(text):

    return json.loads(text)


def _get_inner_column(l, idx):
    
    return l[idx]


if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--loc', action='store')
    FLAGS, _ = parser.parse_known_args()

    graph(FLAGS.loc)
