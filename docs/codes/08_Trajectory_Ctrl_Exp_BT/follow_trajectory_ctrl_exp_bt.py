import time
import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('../utils')

from pyRobotic import robotics
from pyESP32_BT import BluetoothSerialESP32


if __name__ == '__main__':
    # Create a SerialReader instance (adjust port name as needed)
    bt_serial = BluetoothSerialESP32(port='/dev/rfcomm0', n_vars=2)

    # Start reading data from the serial port
    bt_serial.start()

    # Parameters of simulation
    h = 1e-1  # step-size
    tfin = 60  # simulation time
    t = np.arange(0, tfin+h, h)
    N = len(t)

    # Paramters of the robot
    dp = 0.07  # distance of the axis to the point control

    # Initial conditions
    hx0, hy0, varphi0 = [0, 0, 0]
    hx = np.hstack((hx0, np.zeros(N)))
    hy = np.hstack((hy0, np.zeros(N)))
    varphi = np.hstack((varphi0, np.zeros(N)))

    x_0, y_0 = [0.5, 0]
    x = np.hstack((x_0, np.zeros(N)))
    y = np.hstack((y_0, np.zeros(N)))

    # Direct cinematic
    hx[0] = x[0] + dp * np.cos(varphi[0])
    hy[0] = y[0] + dp * np.sin(varphi[0])

    # Desired position (zero)
    hx_d = 0.5 * np.cos(0.1 * t)
    hy_d = 0.5 * np.sin(0.1 * t)

    hx_dp = -0.5 * 0.1 * np.sin(0.1 * t)
    hy_dp = 0.5 * 0.1 * np.cos(0.1 * t)

    # Desired position (eigth)
    # hx_d = 0.5 * np.cos(0.15 * t)
    # hy_d = 0.5 * np.sin(0.30 * t)
    #
    # hx_dp = -0.5 * 0.15 * np.sin(0.15 * t)
    # hy_dp = 0.5 * 0.30 * np.cos(0.30 * t)

    # Reference velocities
    u_ref = np.zeros(N)  # Linear velocity in m/s
    w_ref = np.zeros(N)  # Angular velocity in rad/s

    # Real velocities
    u_real = np.zeros(N)  # Linear velocity in m/s
    w_real = np.zeros(N)  # Angular velocity in rad/s

    # Error
    hx_e = np.zeros(N)
    hy_e = np.zeros(N)

    # Adaptive gain
    K_1 = 2
    K_2 = 2

    # Main control
    for k in range(N):
        start_time = time.time()  # Current time

        hx_e[k] = hx_d[k] - hx[k]
        hy_e[k] = hy_d[k] - hy[k]

        he = np.array([[hx_e[k]],
                       [hy_e[k]]])

        # Desired velocities
        h_dp = np.array([[hx_dp[k]],
                         [hy_dp[k]]])

        # Jacobian matrix
        J = np.array([[np.cos(varphi[k]), -dp * np.sin(varphi[k])],
                      [np.sin(varphi[k]), dp * np.cos(varphi[k])]])

        # Parameter control
        K = np.array([[K_1, 0.0],
                      [0.0, K_2]])

        # Control law
        q_ref = np.linalg.pinv(J) @ (h_dp + K @ he)

        u_ref[k] = q_ref[0][0]
        w_ref[k] = q_ref[1][0]

        bt_serial.write_serial_data(str(u_ref[k]) + "," + str(w_ref[k]))

        u_real[k] = bt_serial.raw_data[0]
        w_real[k] = bt_serial.raw_data[1]

        varphi[k+1] = varphi[k] + h * (w_real[k])

        # Cinematic model
        x[k+1] = x[k] + h * (u_real[k] * np.cos(varphi[k+1]))
        y[k+1] = y[k] + h * (u_real[k] * np.sin(varphi[k+1]))

        # Direct cinematic
        hx[k+1] = x[k+1] + dp * np.cos(varphi[k+1])
        hy[k+1] = y[k+1] + dp * np.sin(varphi[k+1])

        elapsed_time = time.time() - start_time  # Elapsed time

        time.sleep(h - elapsed_time)  # Wait until sampling time is complete

    # Stop the motor
    bt_serial.write_serial_data(str(0) + "," + str(0))

    # Stop the serial reading after simulation time
    bt_serial.stop()

    path_stl = "../utils/stl"
    colors = ['yellow', 'black', 'gray', 'gray', 'white', 'blue']
    unicycle = robotics(path_stl, colors)

    xmin = -3
    xmax = 3
    ymin = -3
    ymax = 3
    zmin = 0
    zmax = 1
    bounds = [xmin, xmax, ymin, ymax, zmin, zmax]
    unicycle.configureScene(bounds)

    unicycle.plotDesiredTrajectory(hx_d, hy_d)
    unicycle.initTrajectory(hx, hy)

    scale = 5
    unicycle.initRobot(x, y, varphi, scale)

    step = 5
    unicycle.startSimulation(step, h)

    plt.figure()
    plt.plot(t, hx_e, label='hx_e(t)')
    plt.plot(t, hy_e, label='hy_e(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('Error(t) [m/s]')

    plt.figure()
    plt.subplot(211)
    plt.plot(t, u_ref, label='u_ref(t)')
    plt.plot(t, u_real, label='u_real(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('mu(t) [m/s]')

    plt.subplot(212)
    plt.plot(t, w_ref, label='w_ref(t)')
    plt.plot(t, w_real, label='w_real(t)')
    plt.grid('on')
    plt.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.ylabel('w(t) [rad/s]')
    plt.show()
