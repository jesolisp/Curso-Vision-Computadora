import cv2

if __name__ == '__main__':
    img = cv2.imread('poligonos.jpg')

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, binary = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY_INV)

    contours, _ = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.1 * cv2.arcLength(cnt, True), True)

        if len(approx) == 1:
            cv2.drawContours(img, [cnt], 0, (0, 0, 255), -1)

    # Display the original image and the image with increased sharpness
    cv2.imshow('Original Image', img)
    # cv2.imshow('Binary Image', binary)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
