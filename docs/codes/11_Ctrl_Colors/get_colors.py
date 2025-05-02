import cv2
import sys
import threading
import numpy as np
import tkinter as tk

from PIL import Image, ImageTk


def onClosing():
    root.quit()
    root.destroy()
    print("Ip-Cam disconected")


def hsv_value(int):
    h_min.set(hmin_slider.get())
    h_max.set(hmax_slider.get())
    s_min.set(smin_slider.get())
    s_max.set(smax_slider.get())
    v_min.set(vmin_slider.get())
    v_max.set(vmax_slider.get())


# Function to stream video and update Tkinter labels
def stream_video():
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Mirror the image vertically (top to bottom)
        # frame = cv2.flip(frame, 0)

        if not ret:
            break

        # Convert the frame from BGR (OpenCV format) to RGB (Tkinter format)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array([h_min.get(), s_min.get(), v_min.get()])
        upper = np.array([h_max.get(), s_max.get(), v_max.get()])

        mask = cv2.inRange(hsv, lowerb=lower, upperb=upper)

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

    cap = cv2.VideoCapture(url)

    if cap.isOpened():
        print("IP-Cam initialized")
    else:
        sys.exit("IP Cam disconnected")

    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", onClosing)
    root.title('Artificial Vision')

    h_min = tk.IntVar()
    h_max = tk.IntVar()
    s_min = tk.IntVar()
    s_max = tk.IntVar()
    v_min = tk.IntVar()
    v_max = tk.IntVar()

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
    hmin_slider = tk.Scale(root, label='Hue Min', from_=0, to=255, orient=tk.HORIZONTAL, command=hsv_value, length=400)
    hmin_slider.grid(row=1, column=0)

    hmax_slider = tk.Scale(root, label='Hue Max', from_=0, to=255, orient=tk.HORIZONTAL, command=hsv_value, length=400)
    hmax_slider.grid(row=1, column=1)

    smin_slider = tk.Scale(root, label='Saturation Min', from_=0, to=255, orient=tk.HORIZONTAL, command=hsv_value, length=400)
    smin_slider.grid(row=2, column=0)

    smax_slider = tk.Scale(root, label='Saturation Max', from_=0, to=255, orient=tk.HORIZONTAL, command=hsv_value, length=400)
    smax_slider.grid(row=2, column=1)

    vmin_slider = tk.Scale(root, label='Value Min', from_=0, to=255, orient=tk.HORIZONTAL, command=hsv_value, length=400)
    vmin_slider.grid(row=3, column=0, padx=20, pady=20)

    vmax_slider = tk.Scale(root, label='Value Max', from_=0, to=255, orient=tk.HORIZONTAL, command=hsv_value, length=400)
    vmax_slider.grid(row=3, column=1, padx=20, pady=20)

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
