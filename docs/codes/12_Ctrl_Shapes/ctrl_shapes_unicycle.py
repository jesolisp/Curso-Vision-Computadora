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


def hsv_value(int):
    h_min.set(hmin_slider.get())
    h_max.set(hmax_slider.get())
    s_min.set(smin_slider.get())
    s_max.set(smax_slider.get())
    v_min.set(vmin_slider.get())
    v_max.set(vmax_slider.get())


def toggle():
    button.config(text=btn_var.get())


def object_detection(raw_image):
    kernel = np.ones((10, 10), dtype=np.uint8)
    is_object = False
    cont = 0
    cx, cy = 0, 0
    cx_d, cy_d = 0, 0

    # Convert the frame from BGR (OpenCV format) to RGB (Tkinter format)
    hsv = cv2.cvtColor(raw_image, cv2.COLOR_BGR2HSV)

    lower = np.array([h_min.get(), s_min.get(), v_min.get()])
    upper = np.array([h_max.get(), s_max.get(), v_max.get()])

    mask = cv2.inRange(hsv, lowerb=lower, upperb=upper)

    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        moments = cv2.moments(cnt)
        area = moments['m00']

        if area > min_area:

            approx = cv2.approxPolyDP(cnt, 0.1 * cv2.arcLength(cnt, True), True)

            if len(approx) >= 4 and len(approx) <= 8:
                cv2.drawContours(raw_image, [cnt], 0, (0, 0, 255), -1)
                cx_d = int(moments['m10'] / moments['m00'])
                cy_d = int(moments['m01'] / moments['m00'])
                cont += 1
            elif len(approx) > 11:
                cv2.drawContours(raw_image, [cnt], 0, (0, 0, 255), -1)
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                cont += 1

    if cont == 2:
        is_object = True
    else:
        is_object = False

    return is_object, mask, cx, cy, cx_d, cy_d


# Function to stream video and update Tkinter labels
def stream_video():
    min_dist = 20

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Mirror the image vertically (top to bottom)
        # frame = cv2.flip(frame, 0)

        if not ret:
            break

        is_obj, mask, cx, cy, cx_d, cy_d = object_detection(frame)

        cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)
        cv2.circle(frame, (cx_d, cy_d), min_dist, (0, 255, 0), 3)

        if is_obj:
            h_x = cx - frame.shape[1] / 2
            h_y = frame.shape[0] / 2 - cy

            hx_d = cx_d - frame.shape[1] / 2
            hy_d = frame.shape[0] / 2 - cy_d

            # Euclidean distance
            distance = np.sqrt((hx_d - h_x)**2 + (hy_d - h_y)**2)

            if distance > min_dist:
                varphi.set(bt_serial.raw_data[2])

                hx_e = hx_d - h_x
                hy_e = hy_d - h_y

                he = np.array([[hx_e],
                               [hy_e]])

                K = np.diag([0.001, 0.001])

                J = np.array([[-np.sin(varphi.get()), -dp * np.cos(varphi.get())],
                              [np.cos(varphi.get()), -dp * np.sin(varphi.get())]])

                # Law control
                qp = np.linalg.pinv(J) @ (K @ he)

                u_ref.set(round(qp[0][0], 3))
                w_ref.set(round(qp[1][0], 3))

            else:
                u_ref.set(0)
                w_ref.set(0)
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
        root.after(0, update_labels, img_tk, mask)


def update_labels(img_tk, mask):
    # Update the GUI from the main thread
    label_left.img_tk = img_tk  # Store a reference to avoid garbage collection
    label_left.configure(image=img_tk)

    # Ensure proper size for label
    label_left.update_idletasks()

    # Update right and bottom frames
    mask = Image.fromarray(mask)
    mask_tk = ImageTk.PhotoImage(image=mask)

    label_right.img_tk = mask_tk  # Store a reference to avoid garbage collection
    label_right.configure(image=mask_tk)

    label_right.update_idletasks()


# Main program starts here
if __name__ == '__main__':
    min_area = 500
    url = 'http://192.168.100.9'
    # url = 'http://192.168.100.38:8080/shot.jpg'

    # cap = cv2.VideoCapture(url)
    cap = cv2.VideoCapture(2)

    # Desired position in pixels
    _, frame = cap.read()
    cx_d = 340
    cy_d = 240

    # Paramters of the robot
    dp = 0.07  # distance of the axis to the point control

    hx_d = cx_d - frame.shape[1] / 2
    hy_d = frame.shape[0] / 2 - cy_d

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

    h_min = tk.IntVar()
    h_max = tk.IntVar()
    s_min = tk.IntVar()
    s_max = tk.IntVar()
    v_min = tk.IntVar()
    v_max = tk.IntVar()

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

    frame_right = tk.Frame(root, width=400, height=400, bg='white')
    frame_right.grid(row=0, column=1, padx=20, pady=20)

    # Create labels to hold the webcam frames
    label_left = tk.Label(frame_left)
    label_left.pack(fill=tk.BOTH, expand=True)  # Ensure label takes the full space

    label_right = tk.Label(frame_right)
    label_right.pack(fill=tk.BOTH, expand=True)  # Ensure label takes the full space

    # Create sliders to control
    hmin_slider = tk.Scale(root, label='Hue Min', from_=0, to=255,
                           orient=tk.HORIZONTAL, command=hsv_value, length=400)
    hmin_slider.grid(row=1, column=0)

    hmax_slider = tk.Scale(root, label='Hue Max', from_=0, to=255,
                           orient=tk.HORIZONTAL, command=hsv_value, length=400)
    hmax_slider.grid(row=1, column=1)

    smin_slider = tk.Scale(root, label='Saturation Min', from_=0, to=255,
                           orient=tk.HORIZONTAL, command=hsv_value, length=400)
    smin_slider.grid(row=2, column=0)

    smax_slider = tk.Scale(root, label='Saturation Max', from_=0, to=255,
                           orient=tk.HORIZONTAL, command=hsv_value, length=400)
    smax_slider.grid(row=2, column=1)

    vmin_slider = tk.Scale(root, label='Value Min', from_=0, to=255,
                           orient=tk.HORIZONTAL, command=hsv_value, length=400)
    vmin_slider.grid(row=3, column=0)

    vmax_slider = tk.Scale(root, label='Value Max', from_=0, to=255,
                           orient=tk.HORIZONTAL, command=hsv_value, length=400)
    vmax_slider.grid(row=3, column=1)

    hmax_slider.set(255)
    smax_slider.set(255)
    vmax_slider.set(255)

    # Start the video streaming in a separate thread to avoid blocking the main GUI loop
    video_thread = threading.Thread(target=stream_video, daemon=True)
    video_thread.start()

    # Start the Tkinter event loop
    root.mainloop()

    # Release the webcam when the program is closed
    cap.release()
