import cv2

from pyzbar import pyzbar


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Mirror the image vertically (top to bottom)
        # frame = cv2.flip(frame, 0)

        if not ret:
            break

        barcodes = pyzbar.decode(frame)

        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            barcode_data = barcode.data.decode('utf-8')
            print(barcode_data)

        cv2.imshow('QR code', frame)

        if(cv2.waitKey(1) & 0xFF == 27):
            break

    print("Camera disconected")

    cap.release()
    cv2.destroyAllWindows()
