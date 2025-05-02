threshold_val = 127
t, binary = cv2.threshold(gray, threshold_val, 250, cv2.THRESH_BINARY)

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
