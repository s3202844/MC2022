import os
import sys
import cv2
import time
import numpy as np
import picar_4wd as fc

from fractions import Fraction
from picamera import PiCamera
from picamera.array import PiRGBArray
from threading import Thread, Lock
from color_detection import *
from mpu6050 import *

global Frame, frame_loaded, cam_lock
global obstacle, cv_lock
global yaw, n_sample, mpu_lock
global cv_task, mpu_start
frame_loaded = -1
obstacle, yaw, n_sample = None, 0, 0
cv_task = 0
mpu_start = False

cam_lock = Lock()
cv_lock = Lock()
mpu_lock = Lock()

if not os.path.exists("results/"):
    os.mkdir("results/")


def capture(camera, stream):
    global Frame, frame_loaded, cam_lock
    cap_timer = time.time()
    for image in camera.capture_continuous(stream, format="bgr", use_video_port=True):
        # print(camera.awb_gains)
        cam_lock.acquire()
        Frame = image.array
        frame_loaded = False
        cam_lock.release()
        stream.truncate(0)
        # print((time.time()-cap_timer)*1000)
        cap_timer = time.time()


def Gyroscope():
    """
    Use Angular velocity from gyroscope MPU6050 to calculate yaw rotation. FIFO
    is enabled to ensure accuracy. Sampling frequency is hard set to 200Hz.

    Author: Haoran Yin
    """
    global yaw, n_sample, mpu_lock
    global i2c_timer, dt, gyroZ_offset, samples
    i2c_timer = time.time()
    dt = 5.0/1000
    gyroZ_offset = 0
    samples = []

    def yaw_rotation(KEY):
        global yaw, n_sample, mpu_start, mpu_lock
        global i2c_timer, dt, gyroZ_offset, samples
        gyro_data = mpu.get_gyro_data()
        mpu_lock.acquire()
        n = n_sample
        mpu_lock.release()
        if n < 1/dt:
            if n == 0:
                samples = [gyro_data['z']*100]
                gyroZ_offset = gyro_data['z']*dt
                mpu_lock.acquire()
                yaw = 0.
                mpu_lock.release()
            else:
                samples += [gyro_data['z']*100]
                gyroZ_offset += gyro_data['z']*dt
            mpu_lock.acquire()
            n_sample += 1
            mpu_lock.release()
            # print(gyro_data['z'])
        else:
            if samples != [] and np.std(samples) > 12:
                mpu_lock.acquire()
                n_sample = 0
                mpu_lock.release()
            else:
                samples = []
                mpu_start = True
                gyroZ = gyro_data['z'] - gyroZ_offset
                mpu_lock.acquire()
                yaw += gyroZ * dt
                if yaw > 360:
                    yaw -= 360
                if yaw < -360:
                    yaw += 360
                mpu_lock.release()
                # print(gyro_data['z'], yaw)
        # print((time.time() - i2c_timer)*1000)
        i2c_timer = time.time()
    mpu = mpu6050(0x68)
    GPIO.add_event_detect(17, GPIO.FALLING, callback=yaw_rotation)


def detect_obstacle(channels=CONTOURS_COMB, debug=False):
    """
    Detect the obstacle and calculate distance.

    Author: Gerlise, Donghang Lyo
    """
    global Frame, frame_loaded, cam_lock
    global obstacle, cv_task, cv_lock
    while True:
        cam_lock.acquire()
        if frame_loaded == -1:
            loaded = True
        else:
            loaded = frame_loaded
        cam_lock.release()
        if not loaded:
            if cv_task == 0:
                cv_task = 1
            cv_timer = time.time()
            cam_lock.acquire()
            frame_loaded = True
            frame = Frame.copy()
            cam_lock.release()
            # give the ROI of the obstacle
            ROI = detect(frame, channels=channels, debug=debug)
            # print((time.time()-cv_timer)*1000)
            cv2.waitKey(10)
        else:
            time.sleep(0.01)


def main():
    """
    The main controlling function. Control the picar to move along two different
    trajectories, and avoid hitting obstacle. 

    Author: Tao Peng
    """
    global obstacle, cv_lock, yaw, n_sample, mpu_lock
    global cv_task, mpu_start
    state = 0
    start_flags = [False for _ in range(4)]
    timer = time.time()
    cache_yaw = 0
    while True:
        if state == 0:
            if cv_task != 0 and mpu_start:
                state = 1
        elif state == 1:
            if start_flags[0] == False:
                start_flags[0] = True
                timer = time.time()
                fc.forward(10)
            else:
                if time.time() - timer > 8:
                    fc.stop()
                    mpu_lock.acquire()
                    cache_yaw = yaw
                    mpu_lock.release()
                    state = 2
        elif state == 2:
            if start_flags[1] == False:
                start_flags[1] = True
                mpu_lock.acquire()
                n_sample = 0
                mpu_lock.release()
            else:
                mpu_lock.acquire()
                rotate = yaw
                mpu_lock.release()
                if rotate != 0:
                    if rotate > -179-2:
                        fc.turn_right(1)
                    elif rotate < -181-2:
                        fc.turn_left(1)
                    else:
                        fc.stop()
                        state = 3
        elif state == 3:
            if start_flags[2] == False:
                start_flags[2] = True
                timer = time.time()
                fc.forward(10)
            else:
                if time.time() - timer > 8:
                    fc.stop()
                    state = 4
        elif state == 4:
            if start_flags[3] == False:
                start_flags[3] = True
                mpu_lock.acquire()
                n_sample = 0
                mpu_lock.release()
            else:
                mpu_lock.acquire()
                rotate = yaw
                mpu_lock.release()
                if rotate != 0:
                    if rotate > -179:
                        fc.turn_right(1)
                    elif rotate < -181:
                        fc.turn_left(1)
                    else:
                        fc.stop()
                        state = 5
        elif state == 5:
            break
        # get the transformation vector from picar to obstacle.
        cv_lock.acquire()
        tVec = obstacle[:] if obstacle != None else None
        cv_lock.release()
        # get the rotatino of picar
        mpu_lock.acquire()
        rotate = yaw
        mpu_lock.release()

        # # if you want to calibrate the mpu, do this
        # # you should calibrate each time you want to turn
        # mpu_lock.acquire()
        # n_sample = 0
        # mpu_lock.release()

        print("===================")
        print("State: ", state)
        print("obstacle:", obstacle)
        print("yaw:", rotate)
        time.sleep(0.01)


if __name__ == "__main__":
    debug = False
    if len(sys.argv) > 1 and sys.argv[1] == "1":
        debug = True

    resolution = (320, 240)
    camera = PiCamera()
    camera.resolution = resolution
    # camera.iso = 100
    camera.contrast = 100
    # camera.saturation = 100
    camera.framerate = 40
    camera.awb_mode = 'off'
    camera.awb_gains = (Fraction(311, 256), Fraction(723, 256))
    # camera.exposure_mode = 'off'
    camera.image_effect = 'saturation'

    stream = PiRGBArray(camera, size=resolution)
    time.sleep(2)

    main_thread = Thread(target=main)
    i2c_thread = Thread(target=Gyroscope)
    cam_thread = Thread(target=capture, args=(camera, stream))
    cv_thread = Thread(target=detect_obstacle, args=(CONTOURS_COMB, debug))

    main_thread.start()
    i2c_thread.start()
    cam_thread.start()
    cv_thread.start()

    main_thread.join()
    i2c_thread.join()
    cam_thread.join()
    cv_thread.join()
