import cv2
import sys
import threading
import numpy as np


# Main program starts here
if __name__ == '__main__':
    min_area = 500
    url = 'http://192.168.100.9'
    threshold_val = 127

    cap = cv2.VideoCapture(0)

    if cap.isOpened():
        print("IP Cam initialized")
    else:
        sys.exit("IP Cam disconnected")

    # Function to stream video and update Tkinter labels
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Mirror the image vertically (top to bottom)
        # frame = cv2.flip(frame, 0)

        if not ret:
            break

        # Convert the frame from BGR (OpenCV format) to RGB (Tkinter format)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        t, binary = cv2.threshold(gray, threshold_val, 255, cv2.THRESH_BINARY)

        kernel = np.ones((5, 5), np.uint8)  # Nucleo
        closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)+

        for cnt in contours:
            moments = cv2.moments(cnt)
            area = moments['m00']

            if area > min_area:
                cv2.drawContours(frame, [cnt], 0, (0, 255, 0), 3)
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                cv2.circle(frame, (cx, cy), 10, (0, 0, 255), -1)

        cv2.imshow('result', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
                break


    # Release the webcam when the program is closed
    cap.release()
