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
from libs.color_detection import *
from libs.mpu6050 import *

global Frame, frame_loaded, cam_lock
global obstacle, cv_lock
global yaw, n_sample, mpu_lock
global cv_task, mpu_start
frame_loaded = -1
obstacle, yaw, n_sample = False, 0, 0
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
            obstacle = detect(frame, channels=channels, debug=debug)
            # print((time.time()-cv_timer)*1000)
            cv2.waitKey(10)
        else:
            time.sleep(0.01)


class Car:
    def __init__(self):
        self.v = 3.7
        self.target_yaw = 0.
        self.avoid_path = 0.5
    
    def start(self):
        global cv_task, mpu_start
        while cv_task == 0 or not mpu_start:
            self.output()

    def run(self, meter, flag=False):
        global obstacle
        timer = time.time()
        fc.forward(10)
        m, n = 0., meter
        while time.time() - timer < self.v * meter:
            if flag and obstacle:
                t = time.time()
                m += (t-timer)/self.v + self.avoid_path
                meter = n-m
                self.avoid()
                fc.forward(10)
                timer = time.time()
            self.output()
        fc.stop()

    def turn(self, degree):
        global mpu_lock, yaw
        direction = 0
        self.target_yaw -= degree
        while True:
            self.output()
            mpu_lock.acquire()
            rotation = yaw
            mpu_lock.release()
            if rotation > self.target_yaw+1:
                if direction != -1:
                    fc.turn_right(10)
                    direction = -1
            elif rotation < self.target_yaw-1:
                if direction != 1:
                    fc.turn_left(10)
                    direction = 1
            else:
                fc.stop()
                break
    
    def avoid(self):
        self.turn(90)
        self.run(0.2)
        self.turn(-90)
        self.run(self.avoid_path)
        self.turn(-90)
        self.run(0.2)
        self.turn(90)
    
    def output(self):
        global cv_lock, obstacle, mpu_lock, yaw
        cv_lock.acquire()
        flag = obstacle
        cv_lock.release()
        mpu_lock.acquire()
        rotate = yaw
        mpu_lock.release()
        print("===================")
        print("obstacle:", flag)
        print("yaw:", rotate)
        time.sleep(0.01)

def main(trajectory, task):
    """
    The main controlling function. Control the picar to move along two different
    trajectories, and avoid hitting obstacle. 

    Author: Tao Peng
    """
    car = Car()
    car.start()
    # car.avoid()
    if trajectory == 1:
        car.run(2, task==2)
        car.turn(180)
        car.run(2, task==2)
        car.turn(180)
    else:
        car.run(2, task==2)
        car.turn(90)
        car.run(1, task==2)
        car.turn(90)
        car.run(2, task==2)
        car.turn(90)
        car.run(1, task==2)
        car.turn(90)


if __name__ == "__main__":
    debug = False
    if len(sys.argv) < 3:
        print("Not enough params:")
        print("\tpython src.py <trajectory 1 or 2> <task 1 or 2> [<visulaize 0 or 1>]")
        sys.exit(-1)
    trajectory = int(sys.argv[1])
    task = int(sys.argv[2])
    if len(sys.argv) > 3 and sys.argv[3] == "1":
        debug = True

    resolution = (320, 240)
    camera = PiCamera()
    camera.resolution = resolution
    # camera.iso = 100
    camera.contrast = 100
    # camera.saturation = 100
    camera.framerate = 40
    # camera.awb_mode = 'off'
    # camera.awb_gains = (Fraction(311, 256), Fraction(723, 256))
    # camera.exposure_mode = 'off'
    camera.image_effect = 'saturation'

    stream = PiRGBArray(camera, size=resolution)
    time.sleep(2)

    i2c_thread = Thread(target=Gyroscope)
    cam_thread = Thread(target=capture, args=(camera, stream))
    cv_thread = Thread(target=detect_obstacle, args=(CONTOURS_COMB, debug))
    main_thread = Thread(target=main, args=(trajectory, task))

    i2c_thread.start()
    cam_thread.start()
    cv_thread.start()
    main_thread.start()

    i2c_thread.join()
    cam_thread.join()
    cv_thread.join()
    main_thread.join()