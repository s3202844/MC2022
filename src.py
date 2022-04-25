import os
import sys
import cv2
import time
import copy
import numpy as np

from picamera import PiCamera
from picamera.array import PiRGBArray
from threading import Thread, Lock
from color_detection import *
from mpu6050 import *

global Frame, frame_loaded, frame_lock
global obstacle, obstacle_lock, yaw, yaw_lock, n_sample, mpu_lock
obstacle, yaw = None, 0
frame_loaded = -1

frame_lock = Lock()
obstacle_lock = Lock()
yaw_lock = Lock()
mpu_lock = Lock()

if not os.path.exists("results/"):
    os.mkdir("results/")


def capture(camera, stream):
    global Frame, frame_loaded, frame_lock
    cap_timer = time.time()
    for image in camera.capture_continuous(stream, format="bgr", use_video_port=True):
        frame_lock.acquire()
        Frame = image.array
        frame_loaded = False
        frame_lock.release()
        stream.truncate(0)
        # print((time.time()-cap_timer)*1000)
        cap_timer = time.time()


def communication():
    """
    Use Angular velocity from gyroscope MPU6050 to calculate yaw rotation. FIFO
    is enabled to ensure accuracy. Sampling frequency is hard set to 200Hz.

    Author: Haoran Yin
    """
    global yaw, yaw_lock, n_sample, mpu_lock
    global i2c_timer, dt, gyroZ_offset
    i2c_timer = time.time()
    dt = 5.0/1000
    mpu_lock.acquire()
    n_sample = 0
    mpu_lock.release()

    def yaw_rotation(KEY):
        global i2c_timer, dt, gyroZ_offset, yaw, yaw_lock, n_sample, mpu_lock
        gyro_data = mpu.get_gyro_data()
        mpu_lock.acquire()
        n = n_sample
        mpu_lock.release()
        if n == 0:
            gyroZ_offset = gyro_data['z']*dt
            yaw_lock.acquire()
            yaw = 0.
            yaw_lock.release()
            mpu_lock.acquire()
            n_sample += 1
            mpu_lock.release()
        elif n < 1/dt:
            gyroZ_offset += gyro_data['z']*dt
            mpu_lock.acquire()
            n_sample += 1
            mpu_lock.release()
        else:
            gyroZ = gyro_data['z'] - gyroZ_offset
            yaw_lock.acquire()
            yaw += gyroZ * dt
            if yaw > 360:
                yaw -= 360
            if yaw < -360:
                yaw += 360
            yaw_lock.release()
        # print((time.time() - i2c_timer)*1000)
        i2c_timer = time.time()
    mpu = mpu6050(0x68)
    GPIO.add_event_detect(17, GPIO.FALLING, callback=yaw_rotation)


def detect_obstacle(channels=CONTOURS_COMB, debug=False):
    """
    Detect the obstacle and calculate distance.

    Author: Gerlise, Donghang Lyo
    """
    global Frame, frame_loaded, frame_lock
    global obstacle, obstacle_lock
    while True:
        frame_lock.acquire()
        if frame_loaded == -1:
            loaded = True
        else:
            loaded = frame_loaded
        frame_lock.release()
        if not loaded:
            cv_timer = time.time()

            frame_lock.acquire()
            frame_loaded = True
            frame = Frame.copy()
            frame_lock.release()

            # give the ROI of the obstacle
            ROI = detect(frame, channels=channels, debug=debug)

            # Todo. calculate the distance

            obstacle_lock.acquire()
            # # Todo. update the distance
            # obstacle = i
            obstacle_lock.release()

            # print((time.time()-cv_timer)*1000)
            if debug:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        else:
            time.sleep(0.01)


def main():
    """
    The main controlling function. Control the picar to move along two different
    trajectories, and avoid hitting obstacle. 

    Author: Tao Peng
    """
    global obstacle, obstacle_lock, yaw, yaw_lock, n_sample, mpu_lock

    while True:
        # get the transformation vector from picar to obstacle.
        obstacle_lock.acquire()
        tVec = obstacle[:] if obstacle != None else None
        obstacle_lock.release()
        # get the rotatino of picar
        yaw_lock.acquire()
        rotate = yaw
        yaw_lock.release()

        # # if you want to calibrate the mpu, do this
        # # you should calibrate each time you want to turn
        # mpu_lock.acquire()
        # n_sample = 0
        # mpu_lock.release()

        print("===================")
        print("obstacle:", obstacle)
        print("yaw:", rotate)
        time.sleep(0.01)


if __name__ == "__main__":
    debug = False
    if len(sys.argv) > 1 and sys.argv[1] == "1":
        debug = True

    camera = PiCamera()
    camera.resolution = (640, 480)
    # camera.iso = 100
    # camera.saturation = 50
    camera.vflip = True
    camera.hflip = True
    camera.framerate = 30
    camera.contrast = 100

    stream = PiRGBArray(camera, size=(640, 480))
    time.sleep(0.5)

    main_thread = Thread(target=main)
    i2c_thread = Thread(target=communication)
    camera_thread = Thread(target=capture, args=(camera, stream))
    cv_thread = Thread(target=detect_obstacle, args=(CONTOURS_COMB, debug))

    main_thread.start()
    i2c_thread.start()
    camera_thread.start()
    cv_thread.start()

    main_thread.join()
    i2c_thread.join()
    camera_thread.join()
    cv_thread.join()
