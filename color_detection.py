import cv2
import numpy as np

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
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Blue color
    low_blue = np.array([50, 206, 130])
    high_blue = np.array([112, 255, 255])
    mask_blue = cv2.inRange(hsv_frame, low_blue, high_blue)
    contours_blue, _ = cv2.findContours(
        mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Green color
    low_green = np.array([30, 76, 86])
    high_green = np.array([56, 255, 206])
    mask_green = cv2.inRange(hsv_frame, low_green, high_green)
    contours_green, _ = cv2.findContours(
        mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Orange color
    low_orange = np.array([0, 76, 232])
    high_orange = np.array([41, 255, 255])
    mask_orange = cv2.inRange(hsv_frame, low_orange, high_orange)
    contours_orange, _ = cv2.findContours(
        mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # pink color
    lower_pink = np.array([157, 211, 94])
    upper_pink = np.array([167, 255, 224])
    mask_pink = cv2.inRange(hsv_frame, lower_pink, upper_pink)
    contours_pink, _ = cv2.findContours(
        mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours = contours_blue + contours_green + contours_pink
    return contours


def find_contours_bgr(frame):
    kernel = np.ones((3, 3), np.uint8)
    _, green = cv2.threshold(frame[:, :, 1], 130, 255, cv2.THRESH_BINARY)
    _, red = cv2.threshold(frame[:, :, 2], 120, 255, cv2.THRESH_BINARY_INV)
    mask_green = np.bitwise_and(red, green)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    # mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    _, green = cv2.threshold(frame[:, :, 1], 75, 255, cv2.THRESH_BINARY_INV)
    _, red = cv2.threshold(frame[:, :, 2], 175, 255, cv2.THRESH_BINARY)
    mask_red = np.bitwise_and(red, green)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
    # mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

    contours_green, _ = cv2.findContours(
        mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_red, _ = cv2.findContours(
        mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    contours = contours_green + contours_red
    return contours


def find_contours(frame, channels=CONTOURS_COMB):
    contours = ()
    if channels == CONTOURS_HSV or channels == CONTOURS_COMB:
        contours += find_contours_hsv(frame)
    if channels == CONTOURS_BGR or channels == CONTOURS_COMB:
        contours += find_contours_bgr(frame)
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
            ROI = [
                max(int(rect[1]-0.5*rect[3]), 0),
                min(int(rect[1]+1.5*rect[3]), frame.shape[1]),
                max(int(rect[0]-0.5*rect[2]), 0),
                min(int(rect[0]+1.5*rect[2]), frame.shape[0])
            ]
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
        if ROI != None:
            cv2.rectangle(results, (ROI[2], ROI[0]),
                          (ROI[3], ROI[1]), (0, 255, 0), 2)
        cv2.imshow("mask", mask)
        cv2.imshow("debug", results)
    return ROI
