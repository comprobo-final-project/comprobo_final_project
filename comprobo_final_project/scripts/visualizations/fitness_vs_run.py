import time
import seaborn

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

seaborn.set()

df = pd.read_csv('logs/log_1493434087.csv', header=None)
plt.plot(df[2])

plt.xlabel('Generations')
plt.ylabel('Fitness')

plt.show()
