import cv2
import numpy as np

from args import Args

CONTOURS_HSV = 0
CONTOURS_BGR = 1
CONTOURS_COMB = 2


def image_colorfulness(image):
    (B, G, R) = cv2.split(image.astype("float"))
    rg = np.absolute(R - G)
    yb = np.absolute(0.5 * (R + G) - B)
    (rbMean, rbStd) = (np.mean(rg), np.std(rg))
    (ybMean, ybStd) = (np.mean(yb), np.std(yb))
    stdRoot = np.sqrt((rbStd ** 2) + (ybStd ** 2))
    meanRoot = np.sqrt((rbMean ** 2) + (ybMean ** 2))
    return stdRoot + (0.3 * meanRoot)


def find_contours_hsv(frame):
    args = Args("config/cv.json")
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Blue color
    lower_blue = np.array(args.hsv_blue[0])
    upper_blue = np.array(args.hsv_blue[1])
    mask_blue = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    contours_blue, _ = cv2.findContours(
        mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Green color
    lower_green = np.array(args.hsv_green[0])
    upper_green = np.array(args.hsv_green[1])
    mask_green = cv2.inRange(hsv_frame, lower_green, upper_green)
    contours_green, _ = cv2.findContours(
        mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Orange color
    lower_orange = np.array(args.hsv_orange[0])
    upper_orange = np.array(args.hsv_orange[1])
    mask_orange = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    contours_orange, _ = cv2.findContours(
        mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # pink color
    lower_pink = np.array(args.hsv_pink[0])
    upper_pink = np.array(args.hsv_pink[1])
    mask_pink = cv2.inRange(hsv_frame, lower_pink, upper_pink)
    contours_pink, _ = cv2.findContours(
        mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.imshow("hsv_mask", mask_orange)

    contours = contours_blue + contours_green + contours_pink
    return contours


def find_contours(frame, channels=CONTOURS_COMB):
    contours = []
    if channels == CONTOURS_HSV or channels == CONTOURS_COMB:
        contours += find_contours_hsv(frame)
    res = []
    for cnt in contours:
        isObject = True
        rect = cv2.minAreaRect(cnt)
        w = rect[1][0] if rect[2] < 45 else rect[1][1]
        h = rect[1][1] if rect[2] < 45 else rect[1][0]
        ratio = w / h if h != 0 else float("inf")
        if cv2.contourArea(cnt) < 100:
            isObject = False
        # if ratio > 6 or ratio < 1:
        #     isObject= False
        if isObject:
            res += [cnt]
    return res


def score_ROIs(frame, contours):
    mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    for cnt in contours:
        rect = cv2.minAreaRect(cnt)
        if rect[1][0] < rect[1][1]:
            exp_rect = (rect[0], (rect[1][0]*3, rect[1][1]), rect[2])
        else:
            exp_rect = (rect[0], (rect[1][0], rect[1][1]*3), rect[2])
        box = cv2.boxPoints(exp_rect)
        box = np.int0(box)
        cv2.drawContours(mask, [box], 0, (255), -1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((10, 10), np.uint8))
    exp_contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    scores = []
    for cnt in exp_contours:
        rect = cv2.boundingRect(cnt)
        score = image_colorfulness(
            frame[rect[1]:rect[1]+rect[3], rect[0]:rect[0]+rect[2]])
        scores += [score]
    return mask, exp_contours, scores


def detect(frame, channels=CONTOURS_COMB, debug=False):
    contours = find_contours(frame, channels)
    mask, exp_contours, scores = score_ROIs(frame, contours)
    max_score = None
    ROI = None
    for i in range(len(scores)):
        cnt = exp_contours[i]
        score = scores[i]
        rect = cv2.boundingRect(cnt)
        dx = rect[1]+rect[3]*0.5-frame.shape[1]*0.5
        dy = rect[0]+rect[2]*0.5-frame.shape[0]*0.5
        dist = (dx**2+dy**2)**0.5
        if (max_score == None or score > max_score) and dist < 100 and score > 75:
            max_score = score
            ROI = rect
    if debug:
        results = np.copy(frame)
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(results, [cnt], 0, (255, 255, 255), 1)
            # cv2.drawContours(results, [box], 0, (0, 255, 0), 1)
        for i in range(len(scores)):
            cnt = exp_contours[i]
            score = scores[i]
            rect = cv2.boundingRect(cnt)
            cv2.rectangle(results, (rect[0], rect[1]),
                          (rect[0]+rect[2], rect[1]+rect[3]), (0, 255, 0), 1)
            cv2.putText(results, str(int(score)), (rect[0], rect[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("mask", cv2.resize(mask, None, fx=2, fy=2))
        cv2.imshow("debug", cv2.resize(results, None, fx=2, fy=2))
    if ROI != None and ROI[2] > 90 and ROI[1]+0.5*ROI[3] > 80:
        return True
    return False
