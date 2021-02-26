import numpy as np    
from collections import namedtuple


LEFT, RIGHT, BACK = 'left', 'right', 'back'


def pol2cart(rho, phi):
    phi = np.deg2rad(phi)
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)


def make_obstacle(dist, dir, last, print_p=True):
    if dir == LEFT:
        angle = -60
    elif dir == RIGHT:
        angle = 60
    else:
        angle = 180

    v = pol2cart(dist, angle)
    x, y = map(lambda x: round(x, 2), [last[0] + v[0], last[1] + v[1]])

    if print_p:
        print('\nCoordinates: (x, y):')
        print('(' + str(x) + ', ' + str(y) + ')\n')
        
    return x, y


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="")
    parser.add_argument('dist', type=int, help='Distance from last')
    parser.add_argument('-l','--last', nargs='+', type=float,
                        help='Last coordinates: x y')
    parser.add_argument('--dir', '-d', type=str, help="Left or right")
    args = parser.parse_args()

    if args.dir.lower() in ('left', 'l'):
        args.dir = LEFT

    else:
        args.dir = RIGHT

    make_obstacle(args.dist, args.dir, last=args.last[:2])