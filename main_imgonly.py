import cv2
# from multiprocessing import Process, Queue
# from ArduinoComms import ArduinoComm
# from AndroidComms import AndroidComm
# from AppletComms import AppletComm
# import numpy as np
# import json
# from time import sleep
from picamera import PiCamera
from ImageRec_imgonly import IMAGEREC
from picamera.array import PiRGBArray


def connect(commsList):
    for comms in commsList:
        comms.connect()


def disconnect(commsList):
    for comms in commsList:
        comms.disconnect()


def listen(msgQueue, com):
    while True:
        msg = com.read()
        msgQueue.put(msg)


if __name__ == '__main__':
    # Initialisation - RPi camera
    print('[RPI_INFO] Initializing Camera.')
    camera = PiCamera()
    camera.brightness = 55
    camera.led = True
    camera.resolution = (390, 240)
    # rawCapture = PiRGBArray(camera, size=(390, 240))
    imageRec = IMAGEREC()

    rawCapture = PiRGBArray(camera, size=(390, 240))
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array
    img1 = image[120:, 0:130, :]
    img2 = image[120:, 130:260, :]
    img3 = image[120:, 260:390, :]
    cv2.imwrite("img1.jpg", img1)
    cv2.imwrite("img2.jpg", img2)
    cv2.imwrite("img3.jpg", img3)
    rect1, leftprediction = imageRec.predict(img1)
    rect2, midprediction = imageRec.predict(img2)
    rect3, rightprediction = imageRec.predict(img3)

rects = []
rects.append(rect1)
rects.append(rect2)
rects.append(rect3)

for i, rect in enumerate(rects):
    if rect != None:
        image = cv2.rectangle(
            image, (i * 120 + rect[0], 120 + rect[1]), (i * 120 + rect[2], 120 + rect[3]), (0, 255, 0), 2)
cv2.imwrite('shouldbeup.jpg', image)
# index = index + 1
# data = {'com': 'Image Taken', 'left': leftprediction,
#         'middle': midprediction, 'right': rightprediction}
# commsList[2].write(json.dumps(data))
print('Left Prediction: ', leftprediction)
print('Middle Prediction: ', midprediction)
print('Right Prediction: ', rightprediction)
