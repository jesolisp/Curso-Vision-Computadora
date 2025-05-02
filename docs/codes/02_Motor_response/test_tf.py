from control.matlab import lsim, tf, pade
import numpy as np
import matplotlib.pyplot as plt

def simulate_fopdt_pade(K, T, L, t, u):
  """
  Simulates the response of a First-Order Plus Dead Time (FOPDT) system
  using a first-order Pade approximation for the time delay.

  Args:
    K: Gain of the process.
    T: Time constant.
    L: Time delay.
    t: Time vector.
    u: Input signal.

  Returns:
    y: Output signal.
  """

  s = tf('s')
  # Pade approximation of time delay
  num_delay, den_delay = pade(L, 1)  # First-order Pade approximation
  delay_approx = tf(num_delay, den_delay)

  # FOPDT transfer function with Pade approximation
  G = (K / (1 + T*s)) * delay_approx

  t, y, _ = lsim(G, u, t)
  return y

# Example usage
K = 1.6022
T = 0.0288
L = 0.2574

# Create time vector and input signal (unit step)
t = np.linspace(0, 10, 100)
u = np.zeros_like(t)
u[t >= 0] = 1

# Simulate the FOPDT system with Pade approximation
y = simulate_fopdt_pade(K, T, L, t, u)

# Plot the results
plt.figure()
plt.plot(t, y)
plt.xlabel("Time")
plt.ylabel("Output")
plt.title("FOPDT System Response (Pade Approximation)")
plt.grid(True)
plt.show()
