import cv2
import sys
import threading
import numpy as np
import tkinter as tk

import sys
sys.path.append('../utils')

from pyzbar import pyzbar
from PIL import Image, ImageTk
from pyESP32_BT import BluetoothSerialESP32


def onClosing():
    root.quit()
    root.destroy()

    # Stop the motor
    bt_serial.write_serial_data(str(0) + "," + str(0))

    # Stop the serial reading after simulation time
    bt_serial.stop()
    print("Ip-Cam disconected")


def toggle():
    button.config(text=btn_var.get())


def qr_detection(raw_image):
    # Initialize variables for center coordinates of robot and object
    cx, cy = 0, 0
    cx_d, cy_d = 0, 0

    # Flags to detect presence of robot and object
    is_robot = False
    is_object = False

    # Preprocess the image to improve detection
    # Convert to grayscale
    gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply binary thresholding
    _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

    # Decode the QR codes from the processed image
    barcodes = pyzbar.decode(thresh)

    # Iterate through all detected barcodes
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        barcode_data = barcode.data.decode('utf-8')

        # If it's a 'robot' barcode
        if barcode_data == 'robot':
            is_robot = True
            cx = x + (w)//2  # Calculate center of the QR code
            cy = y + (h)//2  # Calculate center of the QR code
            cv2.rectangle(raw_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

        # If it's an 'objeto' barcode
        if barcode_data == 'objeto':
            is_object = True
            cx_d = x + (w)//2  # Calculate center of the QR code
            cy_d = y + (h)//2  # Calculate center of the QR code
            cv2.rectangle(raw_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return is_object, is_robot, raw_image, cx, cy, cx_d, cy_d


# Function to stream video and update Tkinter labels
def stream_video():

    while True:
        # Read a frame from the webcam
        cap.open(url)
        ret, frame = cap.read()

        # Mirror the image vertically (top to bottom)
        # frame = cv2.flip(frame, 0)

        if not ret:
            break

        is_obj, is_rbt, frame, cx, cy, cx_d, cy_d = qr_detection(frame)

        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.circle(frame, (cx_d, cy_d), 5, (0, 255, 0), -1)

        if is_obj and is_rbt:
            h_x = cx - frame.shape[1] / 2
            h_y = frame.shape[0] / 2 - cy

            hx_d = cx_d - frame.shape[1] / 2
            hy_d = frame.shape[0] / 2 - cy_d

            varphi.set(bt_serial.raw_data[2])

            hx_e = hx_d - h_x
            hy_e = hy_d - h_y

            he = np.array([[hx_e],
                           [hy_e]])

            K = np.diag([0.0005, 0.0005])

            J = np.array([[-np.sin(varphi.get()), -dp * np.cos(varphi.get())],
                          [np.cos(varphi.get()), -dp * np.sin(varphi.get())]])

            # Law control
            qp = np.linalg.pinv(J) @ (K @ he)

            u_ref.set(round(qp[0][0], 3))
            w_ref.set(round(qp[1][0], 3))

        else:
            u_ref.set(0)
            w_ref.set(0)

        var_u.set("Linear velocity : " + str(u_ref.get()))
        var_w.set("Angular velocity : " + str(w_ref.get()))
        var_Phi.set("Orientation: " + str(varphi.get()))

        if btn_var.get() == 'Running':
            bt_serial.write_serial_data(str(u_ref.get()) + "," + str(w_ref.get()))
        else:
            bt_serial.write_serial_data(str(0) + "," + str(0))

        # Convert the frame to a PhotoImage object (Tkinter-compatible)
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=img)

        # Use root.after to schedule updates to the GUI on the main thread
        root.after(0, update_labels, img_tk)


def update_labels(img_tk):
    # Update the GUI from the main thread
    label_left.img_tk = img_tk  # Store a reference to avoid garbage collection
    label_left.configure(image=img_tk)

    # Ensure proper size for label
    label_left.update_idletasks()


# Main program starts here
if __name__ == '__main__':
    dp = 0.07

    # url = 'http://192.168.100.9'
    url = 'http://192.168.100.38:8080/shot.jpg'

    cap = cv2.VideoCapture(url)
    # cap = cv2.VideoCapture(2)

    if cap.isOpened():
        print("IP-Cam initialized")
    else:
        sys.exit("IP-Cam disconnected")

    # Bluetooth communication
    # Create a SerialReader instance (adjust port name as needed)
    bt_serial = BluetoothSerialESP32(port='/dev/rfcomm0', n_vars=3)

    # Start reading data from the serial port
    bt_serial.start()

    bt_serial.write_serial_data(str(0) + "," + str(0))

    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", onClosing)
    root.title('Artificial Vision')

    u_ref = tk.DoubleVar(root, 0)
    var_u = tk.StringVar(root, "Linear velocity : 0.00")
    label_u = tk.Label(root, textvariable=var_u)
    label_u.grid(row=5, column=0, padx=20, pady=10)

    w_ref = tk.DoubleVar(root, 0)
    var_w = tk.StringVar(root, "Angular velocity : 0.00")
    label_w = tk.Label(root, textvariable=var_w)
    label_w.grid(row=5, column=1, padx=20, pady=10)

    varphi = tk.DoubleVar(root, 0)
    var_Phi = tk.StringVar(root, "Orientation : 0.00")
    label_phi = tk.Label(root, textvariable=var_Phi)
    label_phi.grid(row=5, column=2, padx=20, pady=10)

    btn_var = tk.StringVar(root, 'Run')
    button = tk.Checkbutton(root, text=btn_var.get(), width=12, variable=btn_var,
                            offvalue='Run', onvalue='Running', indicator=False,
                            command=toggle)

    button.grid(row=4, column=0, padx=20, pady=20)

    # Create frames for displaying the video
    frame_left = tk.Frame(root, width=400, height=400, bg='white')
    frame_left.grid(row=0, column=0, padx=20, pady=20)

    # Create labels to hold the webcam frames
    label_left = tk.Label(frame_left)
    label_left.pack(fill=tk.BOTH, expand=True)  # Ensure label takes the full space

    # Start the video streaming in a separate thread to avoid blocking the main GUI loop
    video_thread = threading.Thread(target=stream_video, daemon=True)
    video_thread.start()

    # Start the Tkinter event loop
    root.mainloop()

    # Release the webcam when the program is closed
    cap.release()
