import cv2
import sys
import threading
import numpy as np
import tkinter as tk

import sys
sys.path.append('../utils')

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


def get_threshold_val(val):
    threshold_val.set(slider.get())
    # print(f"Threshold = {threshold_val.get()}")


def toggle():
    button.config(text=btn_var.get())


def object_detection(raw_image):
    kernel = np.ones((10, 10), dtype=np.uint8)
    is_object = False
    cx, cy = 0, 0

    # Convert the frame from BGR (OpenCV format) to RGB (Tkinter format)
    gray = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)

    t, binary = cv2.threshold(gray, threshold_val.get(), 255, cv2.THRESH_BINARY)

    opening = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(opening.copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        moments = cv2.moments(cnt)
        area = moments['m00']

        if area > min_area:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            is_object = True

    return is_object, binary, cx, cy


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

        is_obj, binary, cx, cy = object_detection(frame)

        cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
        cv2.circle(frame, (cx_d, cy_d), 10, (0, 255, 0), -1)

        if is_obj:
            h_x = frame.shape[1] / 2 - cx
            hx_e = hx_d - h_x

            K = 0.0035

            u_ref = 0.2
            w_ref = -K * hx_e
        else:
            u_ref = 0
            w_ref = 0

        if btn_var.get() == 'Running':
            bt_serial.write_serial_data(str(u_ref) + "," + str(w_ref))
        else:
            bt_serial.write_serial_data(str(0) + "," + str(0))

        # Convert the frame to a PhotoImage object (Tkinter-compatible)
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=img)

        # Use root.after to schedule updates to the GUI on the main thread
        root.after(0, update_labels, img_tk, binary)


def update_labels(img_tk, binary):
    # Update the GUI from the main thread
    label_left.img_tk = img_tk  # Store a reference to avoid garbage collection
    label_left.configure(image=img_tk)

    # Ensure proper size for label
    label_left.update_idletasks()

    # Update right and bottom frames
    img_bin = Image.fromarray(binary)
    img_tk_bin = ImageTk.PhotoImage(image=img_bin)

    label_right.img_tk = img_tk_bin  # Store a reference to avoid garbage collection
    label_right.configure(image=img_tk_bin)

    # Ensure proper size for label
    label_right.update_idletasks()


# Main program starts here
if __name__ == '__main__':
    min_area = 500
    # url = 'http://192.168.100.9'
    # url = 'http://192.168.100.38:8080/shot.jpg'
    url = 0

    cap = cv2.VideoCapture(url)

    if cap.isOpened():
        print("IP-Cam initialized")
    else:
        sys.exit("IP-Cam disconnected")

    # Desired position in pixels
    _, frame = cap.read()
    cx_d = int(frame.shape[1] / 2)
    cy_d = int(frame.shape[0] / 2)

    hx_d = 0

    # Bluetooth communication
    # Create a SerialReader instance (adjust port name as needed)
    bt_serial = BluetoothSerialESP32(port='/dev/rfcomm0', n_vars=2)

    # Start reading data from the serial port
    bt_serial.start()

    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", onClosing)
    root.title('Artificial Vision')

    threshold_val = tk.IntVar()
    btn_var = tk.StringVar(root, 'Run')
    button = tk.Checkbutton(root, text=btn_var.get(), width=12, variable=btn_var,
                            offvalue='Run', onvalue='Running', indicator=False, command=toggle)
    button.grid(row=1, column=1)

    # Create fonts with different weights and slants
    label_slider = tk.Label(root)
    label_slider.grid(row=0, padx=20, pady=20)

    # Create frames for displaying the video
    frame_left = tk.Frame(root, width=400, height=400, bg='white')
    frame_left.grid(row=0, column=0, padx=20, pady=20)

    frame_right = tk.Frame(root, width=400, height=400, bg='white')
    frame_right.grid(row=0, column=1, padx=20, pady=20)

    # Create labels to hold the webcam frames
    label_left = tk.Label(frame_left)
    label_left.pack(fill=tk.BOTH, expand=True)  # Ensure label takes the full space

    label_right = tk.Label(frame_right)
    label_right.pack(fill=tk.BOTH, expand=True)  # Ensure label takes the full space

    # Create a slider to control the threshold
    slider = tk.Scale(root, label='Threshold', from_=0, to=255,
                      orient=tk.HORIZONTAL, command=get_threshold_val, length=400)
    slider.grid(row=1, column=0, padx=20, pady=20)

    # Start the video streaming in a separate thread to avoid blocking the main GUI loop
    video_thread = threading.Thread(target=stream_video, daemon=True)
    video_thread.start()

    # Start the Tkinter event loop
    root.mainloop()

    # Release the webcam when the program is closed
    cap.release()
