import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class RangeVisualiser():

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.matshow([])

    def plot_init(self):
        return self.ln


