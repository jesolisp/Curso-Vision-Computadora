import cv2
import sys
import threading
import numpy as np
import tkinter as tk

from PIL import Image, ImageTk


def onClosing():
    root.quit()
    root.destroy()
    print("Ip Cam Disconected")


def get_threshold_val(val):
    threshold_val.set(slider.get())
    print(f"Threshold = {threshold_val.get()}")


# Function to stream video and update Tkinter labels
def stream_video():
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Mirror the image vertically (top to bottom)
        frame = cv2.flip(frame, 0)

        if not ret:
            break

        # Convert the frame from BGR (OpenCV format) to RGB (Tkinter format)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        t, binary = cv2.threshold(gray, threshold_val.get(), 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)  # Nucleo
        closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            moments = cv2.moments(cnt)
            area = moments['m00']

            if area > min_area:
                cv2.drawContours(frame, [cnt], 0, (0, 255, 0), 3)
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)

        # Convert the frame to a PhotoImage object (Tkinter-compatible)
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        img_tk = ImageTk.PhotoImage(image=img)

        # Use root.after to schedule updates to the GUI on the main thread
        root.after(0, update_labels, img_tk, binary, closing)


def update_labels(img_tk, binary, closing):
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

    label_right.update_idletasks()

    img_clos = Image.fromarray(closing)
    img_tk_clos = ImageTk.PhotoImage(image=img_clos)

    label_bottom.img_tk = img_tk_clos  # Store a reference to avoid garbage collection
    label_bottom.configure(image=img_tk_clos)

    label_bottom.update_idletasks()


# Main program starts here
if __name__ == '__main__':
    min_area = 500
    url = 'http://192.168.100.9'
    # url = 'http://192.168.100.38:8080/shot.jpg'

    cap = cv2.VideoCapture(url)

    if cap.isOpened():
        print("IP Cam initialized")
    else:
        sys.exit("IP Cam disconnected")

    root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", onClosing)
    root.title('Artificial Vision')

    threshold_val = tk.IntVar()

    # Create fonts with different weights and slants
    label_slider = tk.Label(root)
    label_slider.grid(row=0, padx=20, pady=20)

    # Create frames for displaying the video
    frame_left = tk.Frame(root, width=400, height=400, bg='white')
    frame_left.grid(row=0, column=0, padx=20, pady=20)

    frame_right = tk.Frame(root, width=400, height=400, bg='white')
    frame_right.grid(row=0, column=1, padx=20, pady=20)

    frame_bottom = tk.Frame(root, width=400, height=400, bg='white')
    frame_bottom.grid(row=1, column=1, padx=20, pady=20)

    # Create labels to hold the webcam frames
    label_left = tk.Label(frame_left)
    label_left.pack(fill=tk.BOTH, expand=True)  # Ensure label takes the full space

    label_right = tk.Label(frame_right)
    label_right.pack(fill=tk.BOTH, expand=True)  # Ensure label takes the full space

    label_bottom = tk.Label(frame_bottom)
    label_bottom.pack(fill=tk.BOTH, expand=True)  # Ensure label takes the full space

    # Create a slider to control the threshold
    slider = tk.Scale(root, label='Threshold', from_=0, to=255, orient=tk.HORIZONTAL, command=get_threshold_val, length=400)
    slider.grid(row=1, column=0)

    # Start the video streaming in a separate thread to avoid blocking the main GUI loop
    video_thread = threading.Thread(target=stream_video, daemon=True)
    video_thread.start()

    # Start the Tkinter event loop
    root.mainloop()

    # Release the webcam when the program is closed
    cap.release()
