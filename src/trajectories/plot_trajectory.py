import matplotlib.pyplot as plt
import csv
import os
from trajectory_vars import *

def read_world(fname):
    with open(os.path.join('csv/', fname + '.csv')) as world_file:
        reader = csv.DictReader(world_file, delimiter=',')
        return [row for row in reader][1:]


def plot_trajectory(fname_world):
    obstacles = read_world(fname_world)
    fig = plt.figure()
    for obstacle in obstacles:
        plt.scatter(obstacle[X], obstacle[Y])
    plt.show()


plot_trajectory('circuit-trees')