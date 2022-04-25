import time

from threading import Lock

def capture_init():
    global Frame, frame_loaded, frame_lock
    frame_loaded = -1
    frame_lock = Lock()
    Frame = None

def capture(camera, stream):
    global Frame, frame_loaded, frame_lock
    cap_timer = time.time()
    for image in camera.capture_continuous(stream, format="bgr", use_video_port=True):
        # print(camera.awb_gains)
        frame_lock.acquire()
        Frame = image.array
        frame_loaded = False
        frame_lock.release()
        stream.truncate(0)
        # print((time.time()-cap_timer)*1000)
        cap_timer = time.time()