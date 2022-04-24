import os
import sys
import cv2
import time
import numpy as np

from picamera import PiCamera
from picamera.array import PiRGBArray
from threading import Thread, Lock
from color_detection import *
from mpu6050 import *

global obstacle, obstacle_lock, yaw, yaw_lock, n_sample, mpu_lock
obstacle, yaw = None, 0

obstacle_lock = Lock()
yaw_lock = Lock()
mpu_lock = Lock()

if not os.path.exists("results/"):
    os.mkdir("results/")


def detect_obstacle(camera, stream, channels=CONTOURS_COMB, debug=False):
    global obstacle, obstacle_lock
    for image in camera.capture_continuous(stream, format="bgr", use_video_port=True):
        frame = image.array
        stream.truncate(0)
        cv_timer = time.time()
        detect(frame, channels=channels, debug=debug)
        obstacle_lock.acquire()
        # obstacle = i
        obstacle_lock.release()
        # print((time.time()-cv_timer)*1000)
        if debug:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


def communication():
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
        elif n <= 1/dt:
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
        # print(time.time() - i2c_timer)
        i2c_timer = time.time()
    mpu = mpu6050(0x68)
    GPIO.add_event_detect(17, GPIO.FALLING, callback=yaw_rotation)


def main():
    global obstacle, obstacle_lock, yaw, yaw_lock, n_sample, mpu_lock

    while True:

        obstacle_lock.acquire()
        tVec = obstacle[:] if obstacle != None else None
        obstacle_lock.release()

        yaw_lock.acquire()
        rotate = yaw
        yaw_lock.release()

        if np.random.rand() < 0.001:
            mpu_lock.acquire()
            n_sample = 0
            mpu_lock.release()

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
    cv_thread = Thread(target=detect_obstacle,
                       args=(camera, stream, CONTOURS_COMB, debug))

    main_thread.start()
    i2c_thread.start()
    cv_thread.start()

    main_thread.join()
    i2c_thread.join()
    cv_thread.join()
