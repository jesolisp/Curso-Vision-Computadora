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

    x_0, y_0 = [0, 0]
    x = np.hstack((x_0, np.zeros(N)))
    y = np.hstack((y_0, np.zeros(N)))

    # Direct cinematic
    hx[0] = x[0] + dp * np.cos(varphi[0])
    hy[0] = y[0] + dp * np.sin(varphi[0])

    # Desired position
    hx_d = 2
    hy_d = 3

    # Reference velocities
    u_ref = np.zeros(N)  # Linear velocity in m/s
    w_ref = np.zeros(N)  # Angular velocity in rad/s

    # Error
    hx_e = np.zeros(N)
    hy_e = np.zeros(N)

    # Adaptive gain
    k_max = 2
    k_1 = 10
    gain = np.zeros(N)

    # Main control
    for k in range(N):
        hx_e[k] = hx_d - hx[k]
        hy_e[k] = hy_d - hy[k]

        he = np.array([[hx_e[k]],
                       [hy_e[k]]])

        # Jacobian matrix
        J = np.array([[np.cos(varphi[k]), -dp * np.sin(varphi[k])],
                      [np.sin(varphi[k]), dp * np.cos(varphi[k])]])

        distance = np.sqrt((hx_d - hx[k])**2 + (hy_d - hy[k])**2)

        gain[k] = k_max / (1 + k_1 * distance)

        # Parameter control
        K = np.array([[gain[k], 0.0],
                      [0.0, gain[k]]])

        # Control law
        q_ref = np.linalg.pinv(J) @ K @ he

        u_ref[k] = q_ref[0][0]
        w_ref[k] = q_ref[1][0]

        varphi[k+1] = varphi[k] + h * (w_ref[k])

        # Cinematic model
        x[k+1] = x[k] + h * (u_ref[k] * np.cos(varphi[k+1]))
        y[k+1] = y[k] + h * (u_ref[k] * np.sin(varphi[k+1]))

        # Direct cinematic
        hx[k+1] = x[k+1] + dp * np.cos(varphi[k+1])
        hy[k+1] = y[k+1] + dp * np.sin(varphi[k+1])

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
    plt.plot(t, hx_e, label='hx_e(t)')
    plt.plot(t, hy_e, label='hy_e(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('e(t) [m]')
    plt.show()

    plt.figure()
    plt.subplot(211)
    plt.plot(t, u_ref, label = 'u_ref(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('mu(t) [m/s]')

    plt.subplot(212)
    plt.plot(t, w_ref, label = 'w_ref(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('w(t) [rad/s]')
    plt.show()

    plt.figure()
    plt.plot(t, gain, label = 'K(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.show()
