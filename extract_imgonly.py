import cv2
import numpy as np

# Image preprocessing


def extractBoundingBox(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    thresh0 = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 51, 1)
    # _, contours,_ = cv2.findContours(thresh0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours, _ = cv2.findContours(
        thresh0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # new opencv returns 2 values only

    # Find bounding box of contours
    bbs = []
    for contour in contours:
        rect = cv2.boundingRect(contour)
        area = rect[2] * rect[3]
        ratio = float(rect[2]) / float(rect[3])

        if area < 100 or area > 15000 or ratio > 1.3:
            continue

        if (ratio >= 0.4 and ratio <= 0.5):
            # 1
            if area < 1000 or area > 5000:
                continue
            bb = (rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])
            bbs.append(bb)

        elif (ratio >= 0.63 and ratio <= 0.75):
            #
            if area < 2000 or area > 7500:
                continue
            bb = (rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])
            bbs.append(bb)

        elif (ratio >= 0.75 and ratio <= 0.95):
            #
            if area < 2500 or area > 7500:
                continue

            bb = (rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])
            bbs.append(bb)

        elif (ratio >= 0.90 and ratio <= 1.1):
            # Up, DOwn, Left, RIGHT sTOP
            if area < 2500 or area > 7500:
                continue

            bb = (rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])
            bbs.append(bb)

        #bb = (rect[0], rect[1], rect[0] + rect[2], rect[1] + rect[3])
        # bbs.append(bb)

    return bbs
