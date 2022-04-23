import os
import sys
import cv2
import time

from color_detection import *

if not os.path.exists("results/"):
    os.mkdir("results/")

cap = cv2.VideoCapture("test/1.avi")


if __name__ == "__main__":
    i = 0
    debug = False
    if len(sys.argv) > 1 and sys.argv[1] == "1":
        debug = True
    while True:
        ret, frame = cap.read()
        if ret:
            pre = time.time()
            detect(frame, channels=CONTOURS_COMB, debug=debug)
            if debug:
                print((time.time()-pre)*1000)
                i += 1
                cv2.imwrite("results/"+str(i)+".png", frame)
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()