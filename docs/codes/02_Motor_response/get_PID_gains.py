import numpy as np


def lambda_tunning(K, tau, delay):
    lamda = 3 * tau
    Kp = tau / (K * (0.5 * delay + lamda))
    Ti = tau
    Td = 0.5 * delay

    kp = Kp * ((Ti + Td) / Ti)
    ti = Ti + Td
    td = Ti * Td / (Ti + Td)

    ki = kp / ti
    kd = kp * td

    return kp, ti, td, ki, kd


if __name__ == '__main__':
    kp, ti, td, ki, kd = lambda_tunning(1.6022, 0.0288, 0.2574)

    print(f"kp = {np.round(kp, 4)}")
    print(f"ti = {np.round(ti, 4)}")
    print(f"td = {np.round(td, 4)}")
    print(f"ki = {np.round(ki, 4)}")
    print(f"kd = {np.round(kd, 4)}")
