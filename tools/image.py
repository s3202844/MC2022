import os
import cv2
import time

from fractions import Fraction
from picamera import PiCamera
from picamera.array import PiRGBArray

if not os.path.exists("images/"):
    os.mkdir("images/")

if __name__ == "__main__":

    resolution = (320, 240)
    camera = PiCamera()
    camera.resolution = resolution
    # camera.iso = 100
    # camera.contrast = 100
    # camera.saturation = 100
    # camera.framerate = 40
    # camera.awb_mode = 'off'
    # camera.awb_gains = (Fraction(311, 256), Fraction(723, 256))
    # camera.exposure_mode = 'off'
    # camera.image_effect = 'saturation'

    stream = PiRGBArray(camera, size=resolution)
    time.sleep(2)

    i = 0
    pre, curr = time.time(), time.time()
    for frame in camera.capture_continuous(stream, format="bgr", use_video_port=True):
        image = frame.array
        cv2.imshow("preview", image)
        stream.truncate(0)
        key = cv2.waitKey(10)
        if key == ord("q"):
            break
        elif key == ord(" "):
            cv2.imwrite("images/"+str(i)+".png", image)
            i += 1
        pre, curr = curr, time.time()
        # print(1/(curr-pre))
    cv2.destroyAllWindows()