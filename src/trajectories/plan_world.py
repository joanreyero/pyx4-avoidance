import numpy as np    
from collections import namedtuple

LEFT, RIGHT, BACK = 'left', 'right', 'back'
DIRECTIONS = [LEFT, RIGHT]

INTRA_OBSTACLE_DIST = 2

fields = ('x', 'y', 'direction', 'add')

Obstacle = namedtuple('Obstacle', fields)


def pol2cart(rho, phi):
    phi = np.deg2rad(phi)
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)


# def make_obstacle(dist, last, rng):

#     angle = np.random.randint(40, 65)
    
#     if last.direction == LEFT:
#         v = pol2cart(dist, -angle)
#         add = True
        
#     elif last.direction == RIGHT:
#         v = pol2cart(dist, angle)
#         dir = LEFT
#         add = True
#     else:
#         v = pol2cart(dist, 180)
#         add = False

#     dir = rng.choice(DIRECTIONS)

#     #v = dist * v #/ np.linalg.norm(v)
#     x, y = last.x + v[0], last.y + v[1]
#     return Obstacle(x=round(x, 2), y=round(y, 2),
#                     direction=dir, add=add)

def make_obstacle(dist, last, rng):

    angle = np.random.randint(40, 65)
    
    if last.direction == LEFT:
        angle = -60
        v = pol2cart(dist, -angle)
        dir = RIGHT
        add = True
        
    elif last.direction == RIGHT:
        angle = 60
        v = pol2cart(dist, angle)
        dir = LEFT
        add = True
    else:
        v = pol2cart(dist, 180)
        add = False
        dir = rng.choice(DIRECTIONS)

    #v = dist * v #/ np.linalg.norm(v)
    x, y = last.x + v[0], last.y + v[1]
    return Obstacle(x=round(x, 2), y=round(y, 2),
                    direction=dir, add=add)


def make_report(out, n, d):
    from os.path import join, isfile
    name = join('world-plans/', 'plan-n_' + str(n) + '-d_' + str(d) + '.txt')
    r = "\n\n============================\n"
    for i, obstacle in enumerate(out):
        r += '\nObstacle: ' + str(i) + ':\n'
        r +='  (' + str(obstacle.x) + ', ' + str(obstacle.y) + ')\n'
        r += '  ' + obstacle.direction + '\n'
        r += '  Add: ' + str(obstacle.add) + '\n'

    with open(name, "a") as text_file:
        text_file.write(r)
        

def plan_world(n, distance, start_cart=(20.00, 0.00), report=True):
    heading = 0
    out = [Obstacle(x=start_cart[0], y=start_cart[1], direction=np.random.choice(DIRECTIONS), add=False)]
    
    rng = np.random.RandomState(seed=0)
    
    for _ in range(n):
#        last = out[rng.randint(0, len(out))]
        last = out[-1]
        obstacle = make_obstacle(distance, last, rng)
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