from os import listdir
from os.path import isfile, join
import pandas as pd
import numpy as np
from analiticsVars import *
from bagReader import AvoidanceBagReader


def make_name(marker, distance):
    """Generate the file name

    Args:
        marker (str or int): set of files
        distance (str or float): stopping distance

    Returns:
        str: file_name
    """
    return str(marker) + '-velocity--dist-' + str(distance) + '.csv'


def is_vel(file):
    return '-velocity--dist-' in file

def parse_vel_for_general(path, file, df):
    df_file = pd.read_csv(path + file, index_col=0)
    return df.append(df_file, ignore_index=True)
    

def parse_dist_for_general(path, file, df):
    df_file = pd.read_csv(path + file)
    if 'velocity--dist' not in file:
        fov_mark = file.find('fov-') + len('fov-')
        fov = int(file[fov_mark : file.find('.csv')])
        df_file_filt = df_file[[ACTIVATION, VELOCITY, POSITION]]
        df_file_filt.insert(2, FOVX, fov)
        df_file_filt = df_file_filt[df_file_filt[POSITION] <= 29]
        return df.append(df_file_filt, ignore_index=True)
    


def get_general_data(marker):
    marker = str(marker)
    path = 'data/'
    files = [f for f in listdir(path) if isfile(join(path, f)) 
             and f[:len(marker)] == marker 
             and f.find('general') == -1]
    df = pd.DataFrame(columns=[VELOCITY, ACTIVATION, FOVX, POSITION, ACTIVATION_GRAD])
    for file in files:
        print(file)
        if not is_vel(file):
            df = parse_dist_for_general(path, file, df)

    df[POSITION] = df[POSITION].apply(lambda a: np.around(a, 2))
    df[VELOCITY] = df[VELOCITY].apply(lambda a: np.around(a, 2))

    df.to_csv(path + marker + '-general.csv')


def get_velocity_data(marker, distance):
    """Parse all the CSV files generated from rosbags
    and get the velocity at the stopping time

    Args:
        marker (int or str): which set of files to parse
        distance (int or str): stopping distance
    """
    path = 'data/'
    marker = str(marker)
    file_name = make_name(marker, distance)
    files = [f for f in listdir(path) if isfile(join(path, f)) 
             and f[:len(marker)] == marker and f != file_name]

    df = pd.DataFrame(columns=[VELOCITY, ACTIVATION, FOVX])

    for file in files:
        file_df = pd.read_csv(path + file)
        temp = file_df.iloc[-1][[VELOCITY, ACTIVATION, POSITION]]
        temp[VELOCITY] = np.round(temp[VELOCITY], 2)
        temp[FOVX] = file[file.find('fov') + 4:file.find('.csv')]
        temp[POSITION] = np.round(temp[POSITION], 2)
        df = df.append(temp, sort=True, ignore_index=True)

    df.to_csv(path + file_name)

def parse_bags(marker, distance):
    path = 'bags/'
    bagfiles = [f for f in listdir(path) if isfile(join(path, f)) 
                and f[:len(marker)] == marker]

    for file in bagfiles:
        reader = AvoidanceBagReader(file[:-4], make_fovs=True, distance=distance)


def main(marker, distance=2, parse_bagsP=False, original_dist=30.13):
    """Main method. Parse the bags (optional) and get the velocity data.

    Args:
        marker (str or int): set of fiels
        distance (float, optional): stopping distance. Defaults to 2.
        parse_bags (bool, optional): whether to parse the bags. 
                                     Defaults to False.
    """
    marker = str(marker)
    distance = str(distance)
    if parse_bagsP:
        parse_bags(marker, original_dist)

    get_velocity_data(marker, distance)
    get_general_data(marker)
    

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Parse the data')
    parser.add_argument('--marker', '-m', default='0', type=str,
                        help='set of data to parse')
    parser.add_argument('--parse-bags', '-b', default=False, type=bool)
    parser.add_argument('--distance', '-d', default=2.0, type=float)
    parser.add_argument('--original_distance', '-o', default=30.13, type=float)
    args = parser.parse_args()

    main(args.marker, parse_bagsP=args.parse_bags, original_dist = args.original_distance)
    #get_general_data('0')