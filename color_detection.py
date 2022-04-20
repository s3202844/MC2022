import cv2
import numpy as np

def multi_color_detector(path):
    while True:
        frame = cv2.imread(path)
        cv2.imshow('sample image', frame)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue color
        low_blue = np.array([50, 206, 130])
        high_blue = np.array([112, 255, 255])
        mask_blue = cv2.inRange(hsv_frame, low_blue, high_blue)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Green color
        low_green = np.array([30, 76, 86])
        high_green = np.array([56, 255, 206])
        mask_green = cv2.inRange(hsv_frame, low_green, high_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Orange color
        low_orange = np.array([0, 76, 232])
        high_orange = np.array([41, 255, 255])
        mask_orange = cv2.inRange(hsv_frame, low_orange, high_orange)
        contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # pink color
        lower_pink = np.array([157, 211, 94])
        upper_pink = np.array([167, 255, 224])
        mask_pink = cv2.inRange(hsv_frame, lower_pink, upper_pink)
        contours_pink, _ = cv2.findContours(mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw all contours
        contours = contours_blue+contours_green+contours_pink
        output = cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
        boxes = []
        for c in contours:
            (x, y, w, h) = cv2.boundingRect(c)
            boxes.append([x,y,x+w,y+h])

        boxes = np.asarray(boxes)
        left, top = np.min(boxes, axis=0)[:2]
        right, bottom = np.max(boxes, axis=0)[2:]
        cv2.rectangle(frame, (left,top), (right,bottom), (0, 255, 0), 2)
        cv2.imshow("Result", frame)

        key = cv2.waitKey(1)
        if key == 27:
            break

if __name__ == '__main__':
    multi_color_detector('results/120.png')