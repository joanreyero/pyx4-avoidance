import xml.etree.ElementTree as ET
import os
import csv
from trajectory_vars import *


root = ET.parse('example.sdf').getroot()


def parse_world(path, fname):
    f = os.path.join(path, fname + '.sdf')
    with open(os.path.join('csv/', fname + '.csv'), 'w') as file:
        writer = csv.writer(file)
        writer.writerow([NAME, SIZE, X, Y])
        for obstacle in OBSTACLES:
            for child in root.findall('world/state/model'):
                if obstacle[NAME] in child.attrib[NAME]:
                    pose = child.find('pose').text.split(' ')[:2]
                    writer.writerow([
                        obstacle[NAME],
                        obstacle[SIZE],
                        pose[0], pose[1]
                    ])


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="")
    path = '/home/joanreyero/Firmware/Tools/sitl_gazebo/worlds'
    parser.add_argument('--path', '-p', type=str, default=path)
    parser.add_argument('--file', '-f', type=str, default='circuit-trees')
    args = parser.parse_args()
    parse_world(args.path, args.file)