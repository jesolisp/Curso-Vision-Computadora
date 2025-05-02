import time

import sys
sys.path.append('../utils')

from pyRobotic import robotics
from pyESP32_BT import BluetoothSerialESP32

if __name__ == '__main__':
    # Create an instance of the BluetoothSerialESP32 class, passing in the Bluetooth serial port
    bt_serial = BluetoothSerialESP32('/dev/rfcomm0', n_vars=2)

    # Start the Bluetooth serial communication
    bt_serial.start()

    # Stop the motor
    bt_serial.write_serial_data(str(0.1) + "," + str(0))

    # Main control
    for k in range(100):
        start_time = time.time()  # Current time

        print(bt_serial.raw_data)

        elapsed_time = time.time() - start_time  # Elapsed time

        time.sleep(0.1 - elapsed_time)  # Wait until sampling time is complete

    # Stop the motor
    bt_serial.write_serial_data(str(0) + "," + str(0))

    # To stop the communication:
    bt_serial.stop()
