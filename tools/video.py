import cv2
import time

from fractions import Fraction
from picamera import PiCamera
from picamera.array import PiRGBArray

if __name__ == "__main__":

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
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
    out = cv2.VideoWriter('output.avi', fourcc, 10, resolution)
    time.sleep(2)

    pre, curr = time.time(), time.time()
    for frame in camera.capture_continuous(stream, format="bgr", use_video_port=True):
        image = frame.array
        cv2.imshow("video", image)
        out.write(image)
        stream.truncate(0)
        key = cv2.waitKey(1)
        if key == ord("q"):
            break
        pre, curr = curr, time.time()
        # print(1/(curr-pre))
    out.release()
    cv2.destroyAllWindows()