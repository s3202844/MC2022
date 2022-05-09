import os
import sys
import cv2
import time

from threading import Thread, Lock
from color_detection import *

global obstacle, yaw, obstacle_lock, yaw_lock
obstacle, yaw = 0, 0

obstacle_lock = Lock()
yaw_lock = Lock()

if not os.path.exists("results/"):
    os.mkdir("results/")


def detect_obstacle(cap, channels=CONTOURS_COMB, debug=False):
    i = 0
    global obstacle, obstacle_lock
    while True:
        ret, frame = cap.read()
        if ret:
            pre = time.time()
            res = detect(frame, channels=channels, debug=debug)
            print(res)
            obstacle_lock.acquire()
            obstacle = i
            # print(obstacle)
            obstacle_lock.release()
            if debug:
                # print((time.time()-pre)*1000)
                i += 1
                cv2.imwrite("results/"+str(i)+".png", frame)
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    break
        else:
            break
    cap.release()


def main():
    global obstacle, obstacle_lock
    while True:
        time.sleep(0.01)
        # obstacle_lock.acquire()
        # print(obstacle)
        # obstacle_lock.release()
        # time.sleep(30)


if __name__ == "__main__":
    debug = False
    if len(sys.argv) > 1 and sys.argv[1] == "1":
        debug = True
    cap = cv2.VideoCapture("test/2.avi")

    main_thread = Thread(target=main)
    cv_thread = Thread(target=detect_obstacle,
                       args=(cap, CONTOURS_COMB, debug))

    main_thread.start()
    cv_thread.start()

    main_thread.join()
    cv_thread.join()

