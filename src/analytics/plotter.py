import matplotlib.pyplot as plt
from analiticsVars import *
import pandas as pd
import numpy as np


path_save = 'figs/'


def get_data(filename):
    return pd.read_csv('data/' + filename + '.csv')


def plot_activation_velocity(filename, fov, msg):
    data = get_data(filename)
    filt_data = data[data[FOVX] == fov]

    plt.title('Activation over different velocities. ' + msg)
    plt.xlabel('Velocity')
    plt.ylabel('Activation')
    plt.scatter(filt_data[VELOCITY], filt_data[ACTIVATION])
    plt.savefig(path_save + filename + '-vel')
    plt.show()


def plot_activation_distance(filename, msg, side=0):
    data = get_data(filename)
    plt.title('Activation over distance to object. ' + msg)
    plt.xlabel('Distance (m)')
    plt.ylabel('Activation')
    plt.scatter(-data[POSITION], data[ACTIVATION])
    plt.savefig(path_save + filename + '-dist')
    plt.show()


def plot_eg_distances():
    files = ['0-xvel-10-yvel-00-fov-120', 
             '0-xvel-20-yvel-00-fov-120', 
             '0-xvel-30-yvel-00-fov-120',
             '0-xvel-40-yvel-00-fov-120']

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharey=True)
    data_1 = get_data(files[0])
    data_2 = get_data(files[1])
    data_3 = get_data(files[2])
    data_4 = get_data(files[3])

    ax1.plot(-data_1[POSITION], data_1[ACTIVATION], linestyle='', marker='o', markersize=2.5)
    ax1.set_xlabel('Distance (m)')
    ax1.set_ylabel('Activation')
    ax1.set_title('v = 1 m/s')

    ax2.plot(-data_2[POSITION], data_2[ACTIVATION], linestyle='', marker='o', markersize=2.5)
    ax2.set_xlabel('Distance (m)')
    ax2.set_ylabel('Activation')
    ax2.set_title('v = 2 m/s')

    ax3.plot(-data_3[POSITION], data_3[ACTIVATION], linestyle='', marker='o', markersize=2.5)
    ax3.set_xlabel('Distance (m)')
    ax3.set_ylabel('Activation')
    ax3.set_title('v = 3 m/s')

    ax4.plot(-data_4[POSITION], data_4[ACTIVATION], linestyle='', marker='o', markersize=2.5)
    ax4.set_xlabel('Distance (m)')
    ax4.set_ylabel('Activation')
    ax4.set_title('v = 4 m/s')

    ax1.set_ylim([0, 2.5])
    ax2.set_ylim([0, 2.5])
    ax3.set_ylim([0, 2.5])
    ax4.set_ylim([0, 2.5])


    fig.suptitle('Activation over distance for different velocities')
    plt.tight_layout()
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    plt.savefig(path_save + 'dist-examples')
    plt.show()


def plot_eg_velocities():
    file = '0-velocity--dist-2'
    data = get_data(file)
    
    data_1 = data[data[FOVX] == 30]
    data_2 = data[data[FOVX] == 60]
    data_3 = data[data[FOVX] == 90]
    data_4 = data[data[FOVX] == 120]
    

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, sharey=True)

    ax1.plot(data_1[VELOCITY], data_1[ACTIVATION], linestyle='', marker='o')
    ax1.set_xlabel('Velocity (m/s)')
    ax1.set_ylabel('Activation')
    ax1.set_title('FOV = 30')

    ax2.plot(data_2[VELOCITY], data_2[ACTIVATION], linestyle='', marker='o')
    ax2.set_xlabel('Velocitu (m/s)')
    ax2.set_ylabel('Activation')
    ax2.set_title('FOV = 60')

    ax3.plot(data_3[VELOCITY], data_3[ACTIVATION], linestyle='', marker='o')
    ax3.set_xlabel('Velocity (m/s)')
    ax3.set_ylabel('Activation')
    ax3.set_title('FOV = 90')

    ax4.plot(data_4[VELOCITY], data_4[ACTIVATION], linestyle='', marker='o')
    ax4.set_xlabel('Velocity (m/s)')
    ax4.set_ylabel('Activation')
    ax4.set_title('FOV = 120')

    ax1.set_ylim([0, 2.5])
    ax2.set_ylim([0, 2.5])
    ax3.set_ylim([0, 2.5])
    ax4.set_ylim([0, 2.5])


    fig.suptitle('Activation over velocity for different FOVs')
    plt.tight_layout()
    fig.tight_layout(rect=[0, 0.03, 1, 0.95])

    plt.savefig(path_save + 'vel-examples')
    plt.show()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--file', '-f', type=str, default='0-xvel-20-yvel-00-fov-12',
                        help='file to plot from')
    parser.add_argument('--type', '-t', default='distance',
                        help='distance, velocity or fov')

    parser.add_argument('--fov', default=120, type=int,
                        help='fov for the velocity graph')

    parser.add_argument('--msg', '-m', default="", type=str,
                        help="message to add to the title")


    args = parser.parse_args()


    if args.type == 'distance' or args.type == 'dist' or args.type == 'd':
        plot_activation_distance(args.file, args.msg)

    elif args.type in ['velocity', 'vel', 'v']:
        plot_activation_velocity(args.file, args.fov, args.msg)

    elif args.type in ['dist-eg', 'd-eg', 'eg-d', 'eg-dist']:
        plot_eg_distances()

    elif args.type in ['vel-eg', 'v-eg', 'eg-v', 'eg-vel']:
        plot_eg_velocities()