import time
import numpy as np
import matplotlib.pyplot as plt

import sys
sys.path.append('../utils')

from pyESP32 import *

if __name__ == '__main__':
    # Create a SerialReader instance (adjust port name as needed)
    esp32 = serialESP32(port='/dev/ttyUSB0', baudrate=115200, n_vars=1)

    # Start reading data from the serial port
    esp32.start()

    esp32.write_serial_data(100)

    h = 0.1
    N = 100

    y = np.zeros(N)

    for k in range(N):
        start_time = time.time() # Current time

        y[k] = esp32.raw_data[0]

        elapsed_time = time.time() - start_time # Elapsed time

        time.sleep(h - elapsed_time) # Wait until sampling time is complete

    # Stop the motor
    esp32.write_serial_data(0)

    # Stop the serial reading after simulation time
    esp32.stop()

    print(y)
    print(np.mean(y))

