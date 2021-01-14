import numpy as np    
from collections import namedtuple

LEFT, RIGHT, BACK = 'left', 'right', 'back'
DIRECTIONS = [LEFT, RIGHT]

INTRA_OBSTACLE_DIST = 2

fields = ('x', 'y', 'direction')

Obstacle = namedtuple('Obstacle', fields, defaults=(None,) * len(fields))

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, np.rad2deg(phi))


def pol2cart(rho, phi):
    phi = np.deg2rad(phi)
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)


def get_angle_direction(last_dir):
    if last_dir == LEFT:
        return 45
    elif last_dir == RIGHT:
        return -45
    else:
        return -1


def make_obstacle(distance, last):
    last_mag, last_ang = cart2pol(last.x, last.y)
    new_mag = last_mag + distance

    new_ang = last_ang + get_angle_direction(last.direction)
    new_x, new_y = pol2cart(new_mag, new_ang)

    new_dir = np.random.choice(DIRECTIONS)
    return Obstacle(x=round(new_x, 2), 
                    y=round(new_y, 2),
                    direction=new_dir)
    

def make_report(out, n, d):
    from os.path import join, isfile
    name = join('world-plans/', 'plan-n_' + str(n) + '-d_' + str(d) + '.txt')
    r = "\n\n============================\n"
    for i, obstacle in enumerate(out):
        r += '\nObstacle: ' + str(i) + ':\n'
        r +='  (' + str(obstacle.x) + ', ' + str(obstacle.y) + ')\n'
        r += '  ' + obstacle.direction + '\n'

    with open(name, "a") as text_file:
        text_file.write(r)

def plan_world(n, distance, start_cart=(1.00, 0.00), report=True):
    heading = 0
    out = [Obstacle(x=start_cart[0], y=start_cart[1], direction=np.random.choice(DIRECTIONS))]

    for _ in range(n):
        obstacle = make_obstacle(distance, out[-1])
        out.append(obstacle)

    if report:
        make_report(out, n, distance)

    return out
        

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="")
    parser.add_argument('--number', '-n', type=int, default=5)
    parser.add_argument('--distance', '-d', type=int, default=7)
    args = parser.parse_args()
    plan_world(args.number, args.distance)