import cv2
import sys
import numpy as np

from args import Args


def nothing(x):
    pass


args = Args("config\cv.json")

# Create a window
cv2.namedWindow('panel', cv2.WINDOW_NORMAL)
# cv2.setWindowProperty('panel', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

# create trackbars for color change
# Hue is from 0-179 for Opencv
cv2.createTrackbar('HMin', 'panel', args.hsv_blue[0][0], 180, nothing)
cv2.createTrackbar('HMax', 'panel', args.hsv_blue[1][0], 180, nothing)
cv2.createTrackbar('SMin', 'panel', args.hsv_blue[0][1], 255, nothing)
cv2.createTrackbar('SMax', 'panel', args.hsv_blue[1][1], 255, nothing)
cv2.createTrackbar('VMin', 'panel', args.hsv_blue[0][2], 255, nothing)
cv2.createTrackbar('VMax', 'panel', args.hsv_blue[1][2], 255, nothing)
cv2.createTrackbar('color', 'panel', 0, 3, nothing)
color_flag = 0
color = args.hsv_blue

# Output Image to display
cap = cv2.VideoCapture("test/2.avi")

while True:
    ret, img = cap.read()
    if ret:
        if color_flag != cv2.getTrackbarPos('color', 'panel'):
            color_flag = cv2.getTrackbarPos('color', 'panel')
            if color_flag == 0:
                color = args.hsv_blue
            elif color_flag == 1:
                color = args.hsv_green
            elif color_flag == 2:
                color = args.hsv_orange
            elif color_flag == 3:
                color = args.hsv_pink
            cv2.setTrackbarPos('HMin', 'panel', color[0][0])
            cv2.setTrackbarPos('SMin', 'panel', color[0][1])
            cv2.setTrackbarPos('VMin', 'panel', color[0][2])
            cv2.setTrackbarPos('HMax', 'panel', color[1][0])
            cv2.setTrackbarPos('SMax', 'panel', color[1][1])
            cv2.setTrackbarPos('VMax', 'panel', color[1][2])
        # get current positions of all trackbars
        color[0][0] = cv2.getTrackbarPos('HMin', 'panel')
        color[0][1] = cv2.getTrackbarPos('SMin', 'panel')
        color[0][2] = cv2.getTrackbarPos('VMin', 'panel')

        color[1][0] = cv2.getTrackbarPos('HMax', 'panel')
        color[1][1] = cv2.getTrackbarPos('SMax', 'panel')
        color[1][2] = cv2.getTrackbarPos('VMax', 'panel')

        # Set minimum and max HSV values to display
        lower = np.array(color[0])
        upper = np.array(color[1])

        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(img, img, mask=mask)
    else:
        break

    # Display output image
    cv2.imshow('image', output)

    # Wait longer to prevent freeze for videos.
    if cv2.waitKey(30) & 0xFF == ord('q'):
        break
args.save()
# Release resources
cap.release()
