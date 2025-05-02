import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('../utils')

from pyRobotic import robotics

if __name__ == '__main__':
    # Parameters of simulation
    h = 1e-1  # step-size
    tfin = 65  # simulation time
    N = np.int64(np.ceil((tfin-h)/h))
    t = h + np.arange(0, N)*h

    # Initial conditions
    hx0, hy0, varphi0 = [0, 0, 0]
    hx = np.hstack((hx0, np.zeros(N)))
    hy = np.hstack((hy0, np.zeros(N)))
    varphi = np.hstack((varphi0, np.zeros(N)))

    mu = 0.1 * np.ones(N)  # Linear velocity [m/s]
    w = 0.1 * np.ones(N)  # Angular velocity [m/s]

    for k in range(N):
        hx[k+1] = hx[k] + h * (mu[k] * np.cos(varphi[k]))
        hy[k+1] = hy[k] + h * (mu[k] * np.sin(varphi[k]))
        varphi[k+1] = varphi[k] + h * (w[k])

    plt.figure()
    plt.plot(hx, hy)
    plt.show()

    path_stl = "../utils/stl"
    colors = ['yellow', 'black', 'gray', 'gray', 'white', 'blue']
    unicycle = robotics(path_stl, colors)

    xmin = -1.5
    xmax = 1.5
    ymin = -1
    ymax = 3
    zmin = 0
    zmax = 1
    bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
    unicycle.configureScene(bounds)

    unicycle.initTrajectory(hx, hy)

    scale = 3
    unicycle.initRobot(hx, hy, varphi, scale)

    step = 5
    unicycle.startSimulation(step, h)
