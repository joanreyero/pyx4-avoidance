import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class RangeVisualiser():

    def __init__(self):
        self.fig = plt.figure()

        self.init_array = np.zeros((100, 100))
        self.img = plt.imshow(self.init_array, animated=True)

    def update(self, frames):
        self.img.set_array(np.random.normal(size=(100, 100)))
        return self.img


if __name__ == '__main__':
    rv = RangeVisualiser()
    ani = FuncAnimation(rv.fig, rv.update, interval=50, blit=True)
    plt.show()
    # import numpy as np
    # import matplotlib.pyplot as plt
    # import matplotlib.animation as animation

    # fig = plt.figure()


    # def f(x, y):
    #     return np.sin(x) + np.cos(y)

    # x = np.linspace(0, 2 * np.pi, 120)
    # y = np.linspace(0, 2 * np.pi, 100).reshape(-1, 1)

    # im = plt.imshow(f(x, y), animated=True)


    # def updatefig(*args):
    #     global x, y
    #     x += np.pi / 15.
    #     y += np.pi / 20.
    #     im.set_array(f(x, y))
    #     return im,

    # ani = animation.FuncAnimation(fig, updatefig, interval=50, blit=True)
    # plt.show(block=True)



