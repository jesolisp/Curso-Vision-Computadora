import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

from scipy.optimize import curve_fit


def fopdt(t, Kp, tau, theta):
    """
     Kp: the gain of the process
     tau: time constant of the process
     theta: dead time of the system
    """

    y = np.zeros_like(t)
    t_ = np.zeros_like(t)

    # Calculate the difference in u values
    delta_u = np.diff(u, prepend=0)

    n = np.where(delta_u != 0)[0][0]

    t_[n:] = t[:-n]

    mask = t_ > theta

    y[mask] = Kp * u[mask] * (1 - np.exp(-(t_[mask] - theta) / tau))

    return y


def tf_response(sys, t, u):
    # _, yout = ctrl.input_output_response(ctrl.tf2ss(sys), T=t, U=u)
    _, yout = ctrl.step_response(ctrl.tf2ss(sys), t)

    return yout


def pid_tuning_fopdt(K, T, L, method='Ziegler-Nichols', alpha=None):
    """
    Tune the PID controller parameters for a FOPDT model using a specified method.

    :param K: Process Gain
    :param tau: Process Time Constant
    :param L: Process Dead Time
    :param method: Tuning method ('Ziegler-Nichols', 'IMC', 'Cohen-Coon', 'Lambda')
    :param alpha: Scaling factor for desired closed-loop time constant (for Lambda method)
    :return: (Kp, Ki, Kd)
    """

    if method == 'Ziegler-Nichols':
        Kp = (1.2 / K) * (T / L)
        Ti = 2 * L
        Td = (1/2) * L
    elif method == 'IAE':
        Kp = (1.435 / K) * np.power(L / T, -0.921)
        Ti = (T / 0.878) * np.power(L / T, 0.749)
        Td = 0.482 * T * np.power(L / T, 1.137)
    elif method == 'Haalman':
        Kp = (2 * T) / (3 * K * L)
        Ti = T
        Td = 0
    elif method == 'Lambda':
        if alpha is None:
            raise ValueError("Alpha value must be provided for Lambda tuning method")

        # Calculate the desired closed-loop time constant: λ = α * τ
        lamda = alpha * T

        # Calculate the PID gains using the Lambda tuning method
        K_p = T / (K * (L / 2 + lamda))

        # Integral Time (T_i) is simply λ
        T_i = T

        # Derivative Time (T_d) is λ / K
        T_d = L / 2

        # Proportional Gain (Kp)
        Kp = K_p * ((T_i + T_d) / T_i)

        # Integral Time (Ti)
        Ti = T_i + T_d

        # Derivative Time (Td)
        Td = (T_i * T_d) / (T_i + T_d)

    else:
        raise ValueError("Unknown method. Choose 'Ziegler-Nichols', 'IAE', 'Haalman', or 'Lambda'")

    return Kp, Ti, Td


if __name__ == '__main__':
    with open('input_response.npy', 'rb') as f:
        u = np.load(f)
        y = np.load(f)
        t = np.load(f)
        h = np.load(f)

    y = np.nan_to_num(y)

    # Initial parameter guesses: Kp, tau, theta
    initial_guesses = [np.random.rand(), np.random.rand(), np.random.rand()]

    # Curve fitting
    popt, _ = curve_fit(fopdt, t, y, p0=initial_guesses)
    K, T, L = popt

    print(f"K = {np.round(K, 4)}")
    print(f"T (time constant)= {np.round(T, 4)}")
    print(f"L (dead time) = {np.round(L, 4)}")

    # s = ctrl.tf('s')
    # Gs = (K / (1 + s * T)) * ((1 - (L / 2) * s) / (1 + (L / 2) * s))

    # Pade approximation of time delay
    # num_delay, den_delay = ctrl.pade(L, 1)  # First-order Pade approximation
    # delay_approx = ctrl.tf(num_delay, den_delay)

    # FOPDT transfer function with Pade approximation
    # Gs = (K / (1 + T * s)) * delay_approx

    # y_hat = tf_response(Gs, t, u)

    # Calculate PID parameters using Ziegler-Nichols
    alpha = 3  # Desired scaling factor for closed-loop time constant
    Kp, Ti, Td = pid_tuning_fopdt(K, T, L, method='Lambda', alpha=alpha)

    print(f"Proportional gain (Kp): {np.round(Kp, 4)}")
    print(f"Integral time (Ti): {np.round(Ti, 4)}")
    print(f"Derivative time (Td): {np.round(Td, 4)}")
    #
    # Cs = Kp * (1 + (1 / (Ti * s)) + Td * s)
    #
    # sys_cl = ctrl.feedback(Gs * Cs, 1)
    #
    # y_hat = tf_response(sys_cl, t, u)
    #
    # plt.figure()
    # plt.plot(t, y_hat, label='y(t)')
    # # plt.plot(t, u, label='u(t)')
    # plt.legend(loc='upper left')
    #
    # plt.show()
