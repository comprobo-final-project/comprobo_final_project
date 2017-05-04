import time
import seaborn

import pandas as pd
import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

seaborn.set()

def graph(log_location):
    df = pd.read_csv(log_location, header=None)
    b, a = signal.butter(3, 0.03)
    y = signal.filtfilt(b, a, df[2])
    plt.plot(df[2], alpha=0.3)
    plt.plot(y)

    plt.xlabel('Generations')
    plt.ylabel('Fitness')

    plt.show()


if __name__ == "__main__":

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--loc', action='store')
    FLAGS, _ = parser.parse_known_args()

    graph(FLAGS.loc)
