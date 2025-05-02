import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('../utils')

from pyRobotic import robotics

if __name__ == '__main__':
    # Parameters of simulation
    h = 1e-1  # step-size
    tfin = 30  # simulation time
    t = np.arange(0, tfin+h, h)
    N = len(t)

    # Paramters of the robot
    dp = 0.07  # distance of the axis to the point control

    # Initial conditions
    hx0, hy0, varphi0 = [0, 0, 0]
    hx = np.hstack((hx0, np.zeros(N)))
    hy = np.hstack((hy0, np.zeros(N)))
    varphi = np.hstack((varphi0, np.zeros(N)))

    # Desired position
    hx_d = 1
    hy_d = 1

    # Desired orientation
    varphi_d = np.pi/2

    # Reference velocities
    u_ref = np.zeros(N)  # Linear velocity in m/s
    w_ref = np.zeros(N)  # Angular velocity in rad/s

    # Error
    l = np.zeros(N)
    rho = np.zeros(N)
    theta = np.zeros(N)

    # Adaptive gain
    K_1 = 0.2
    K_2 = 0.2

    # Main control
    for k in range(N):
        l[k] = np.sqrt((hx_d - hx[k])**2 + (hy_d - hy[k])**2)
        rho[k] = np.arctan2(hy_d - hy[k], hx_d - hx[k]) - varphi[k]
        theta[k] = np.arctan2(hy_d - hy[k], hx_d - hx[k]) - varphi_d

        # Control law
        u_ref[k] = K_1 * l[k] * np.cos(rho[k])
        w_ref[k] = K_2 * rho[k] + K_1 * np.cos(rho[k]) * np.sin(rho[k]) * ((rho[k] + theta[k]) / rho[k])

        varphi[k+1] = varphi[k] + h * (w_ref[k])

        # Cinematic model
        hx[k+1] = hx[k] + h * (u_ref[k] * np.cos(varphi[k+1]))
        hy[k+1] = hy[k] + h * (u_ref[k] * np.sin(varphi[k+1]))

    path_stl = "../utils/stl"
    colors = ['yellow', 'black', 'gray', 'gray', 'white', 'blue']
    unicycle = robotics(path_stl, colors)

    xmin = -1.5
    xmax = 3
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

    plt.figure()
    plt.subplot(211)
    plt.plot(t, l, label='u_ref(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('Error(t) [m/s]')

    plt.subplot(212)
    plt.plot(t, rho, label='rho(t)')
    plt.plot(t, theta, label='theta(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('Error(t) [rad/s]')
    plt.show()

    plt.figure()
    plt.subplot(211)
    plt.plot(t, u_ref, label='u_ref(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('mu(t) [m/s]')

    plt.subplot(212)
    plt.plot(t, w_ref, label='w_ref(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('w(t) [rad/s]')
    plt.show()
