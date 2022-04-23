import os
import re
import cv2
import time
import numpy as np

class Box():
    def __init__(self, contour, rect):
        self.center = [rect[0][0], rect[0][1]]
        self.rects = [rect]
        self.contours = [contour]
        self.maxWidth = max(rect[1])
    
    def append(self, contour, rect):
        dx = abs(rect[0][0]-self.center[0])
        dy = abs(rect[0][1]-self.center[1])
        dist = (dx**2+dy**2)**0.5
        if dist > max(self.maxWidth, max(rect[1])):
            return False
        n = len(self.rects)
        self.center[0] = self.center[0]*n/(n+1) + rect[0][0]/(n+1)
        self.center[1] = self.center[1]*n/(n+1) + rect[0][1]/(n+1)
        self.rects += [rect]
        self.contours += [contour]
        self.maxWidth = max(self.maxWidth, max(rect[1]))
        return True

def getMask(frame, color, debug=False):
    if color == "green":
        _, green = cv2.threshold(frame[:, :, 1], 130, 255, cv2.THRESH_BINARY)
        _, red = cv2.threshold(frame[:, :, 2], 120, 255, cv2.THRESH_BINARY_INV)
    elif color == "red":
        _, green = cv2.threshold(frame[:, :, 1], 75, 255, cv2.THRESH_BINARY_INV)
        _, red = cv2.threshold(frame[:, :, 2], 175, 255, cv2.THRESH_BINARY)
    kernel = np.ones((3, 3), np.uint8)
    mask = np.bitwise_and(red, green)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    if debug:
        cv2.imshow(color, mask)
    return mask

def findBar(mask):
    raw_contours, _ = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours, rects  = [], []
    for cnt in raw_contours:
        isObject = True
        rect = cv2.minAreaRect(cnt)
        w = rect[1][0] if rect[2] < 45 else rect[1][1]
        h = rect[1][1] if rect[2] < 45 else rect[1][0]
        ratio = w / h if h != 0 else float("inf")
        if cv2.contourArea(cnt) < 120:
            isObject= False
        if ratio > 6 or ratio < 2:
            isObject= False
        if isObject:
            contours += [cnt]
            rects += [rect]
    return contours, rects


def findBox(frame, debug=False):
    boxes = []
    green_mask = getMask(frame, color="green", debug=debug)
    green_cnt, green_rects = findBar(green_mask)
    red_mask = getMask(frame, color="red", debug=debug)
    red_cnt, red_rects = findBar(red_mask)
    cnts = green_cnt + red_cnt
    rects = green_rects + red_rects
    # if cnts != []:
        # boxes += [Box(cnts[0], rects[0])]
        # for i in range(1, len(cnts)):
        #     flag = False
        #     for box in boxes:
        #         if box.append(cnts[i], rects[i]):
        #             flag = True
        #             break
        #     if not flag:
        #         boxes += [Box(cnts[i], rects[i])]
    if debug:
        # show = frame.copy()
        for cnt in cnts:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(frame, [cnt], 0, (255, 255, 255), 1)
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 1)
        cv2.imshow("debug", frame)
    return (200, 200), (400, 400)


if __name__ == "__main__":
    if not os.path.exists("results/"):
        os.mkdir("results/")

    cap = cv2.VideoCapture("test/0.avi")

    i = 0
    while(True):
        ret, frame = cap.read()
        if ret:
            t0 = time.time()
            p0, p1 = findBox(frame, True)
            print(time.time()-t0)
            cv2.rectangle(frame, p0, p1, (0, 255, 0))
            cv2.imshow("frame", frame)
            cv2.imwrite("results/"+str(i)+".png", frame)
            i += 1
            if cv2.waitKey(100) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()