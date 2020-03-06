from multiprocessing import Process, Queue
from time import sleep

from ArduinoComms import ArduinoComm
from AndroidComms import AndroidComm
from AppletComms import AppletComm
from ImgRaw import SendRawImages

import numpy as np
import os
import json
import sys
import signal
from datetime import datetime
import argparse

from ImageRec import IMAGEREC
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray

# (Thanks Kaishuo!) Setup to handle cases where serial port jumps to ACM1
# Run this command via SSH: $ sudo python3 main_noIR.py --port /dev/ttyACM0 (or ttyACM1 - check with ls /dev/ttyACM*)
parser = argparse.ArgumentParser(description='MDP RPi Module')
parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='Arduino Serial port')
args = parser.parse_args()
arduino_port = args.port

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
    ## Initialisation - RPi camera - a bloody mess
    print('[RPI_INFO] Initializing Camera.')
    camera = PiCamera()
    camera.resolution = (416, 240)
    camera.framerate = 30
    camera.start_preview()
    print('[RPI_INFO] Warming up camera...')
    sleep(2)
    print('[RPI_INFO] Camera warmed up and ready')
    imageRec = IMAGEREC()

    ## Gracefully shut down camera - in case we need it
    # def sigterm_intercept(sig, frame):
    #     print("Launching Graceful Shutdown")
    #     print("Gracefully shutting down app")
    #     if camera is not None:
    #         print("Closing Camera")
    #         camera.close()
    #         print("Camera Closed")
    #     sys.exit(0)
    # signal.signal(signal.SIGINT, sigterm_intercept)

    ## Set up message logs
    run_timestamp = datetime.now().isoformat()
    os.makedirs('logs', exist_ok=True)
    logfile = open(os.path.join('logs', 'rpilog_' + run_timestamp + '.txt'), 'a+')

    ## Initialisation - RPi Comms
    commsList = []
    commsList.append(ArduinoComm(port=arduino_port))
    commsList.append(AndroidComm())
    commsList.append(AppletComm())
    connect(commsList)

    ARDUINO = 0
    ANDROID = 1
    APPLET = 2

    msgQueue = Queue()
    arduinoListener = Process(target=listen, args=(msgQueue, commsList[ARDUINO]))
    androidListener = Process(target=listen, args=(msgQueue, commsList[ANDROID]))
    appletListener = Process(target=listen, args=(msgQueue, commsList[APPLET]))

    arduinoListener.start()
    androidListener.start()
    appletListener.start()

    # Clear previous images
    imagedir = "/home/pi/RPi_v2/correct_images/"
    test = os.listdir(imagedir)

    for f in test:
        if f.endswith(".jpg"):
            os.remove(os.path.join(imagedir, f))

    ## Initialise variables
    running = True
    exploring = False
    obsHex = ''
    expHex = ''
    imgs = ''
    # index = 0
    correctImages = [] # Array to send raw image data
    correctIdx = []

    try:
        while running:
            message = msgQueue.get()

            if message == None:
                continue

            try:
                logfile.write(message)
            except Exception as e:
                print('[LOGFILE_ERROR] Logfile Write Error: %s' % str(e))

            msgSplit = message.split(';')  # Try without semi-colon

            for i, value in enumerate(msgSplit):
                # Skip the first empty string
                if i == 0:
                    continue
                msg = json.loads(value)
                com = msg['com']

                ## W, A, D: From Android or Applet
                if com == 'W':
                    # Move forward
                    commsList[ARDUINO].write('W')
                    commsList[ANDROID].write('{"com": "statusUpdate", "status": "Moving forward"}')

                elif com == 'A':
                    # Turn left
                    commsList[ARDUINO].write('A')
                    commsList[ANDROID].write(';{"com": "statusUpdate", "status": "Turning left"}')

                elif com == 'D':
                    # Turn right
                    commsList[ARDUINO].write('D')
                    commsList[ANDROID].write(';{"com": "statusUpdate", "status": "Turning right"}')

                ## Exploration and Fastest Path: From Android and Applet
                elif com == 'ex':
                    # Start Explore
                    if exploring == False:
                        commsList[ANDROID].write(';{"com": "statusUpdate", "status": "Exploring"}')
                        commsList[APPLET].write('{"com": "statusUpdate", "status": "Exploring"}')
                        exploring = True

                elif com == 'fp':
                    # Start Fastest Path
                    commsList[ANDROID].write(';{"com": "statusUpdate", "status": "Running Fastest Path"}')
                    commsList[APPLET].write('{"com": "statusUpdate", "status": "Running Fastest Path"}')
                ## Additional command: Applet send path to Arduino
                elif com == 'fpath':
                    commsList[ARDUINO].write(msg['path'])

                ## Sensor Data: From Arduino
                elif com == 'SD':
                    commsList[APPLET].write(json.dumps(msg))

                ## Way point and Starting point: From Android
                ## Check if WM might want to send without formatting data
                elif com == 'wayPoint':
                    # Received waypoint
                    wp = msg['wayPoint']
                    data = {'com': 'wayPoint', 'wayPoint': wp}
                    commsList[APPLET].write(json.dumps(data))

                elif com == 'startingPoint':
                    # Received robot starting position
                    sp = msg['startingPoint']
                    data = {'com': 'startingPoint', 'startingPoint': sp}
                    commsList[APPLET].write(json.dumps(data))

                elif com == 'K':
                    # Force get sensor data from arduino
                    commsList[ARDUINO].write('K')

                ## R, F, C: all calibration - from Applet or Arduino
                elif com == 'R':
                    if msg['from'] == 'Applet':
                        commsList[ARDUINO].write('R')
                    elif msg['from'] == 'Arduino':
                        commsList[APPLET].write('{"com":"statusUpdate", "status":"Finish Calibrate"}')

                elif com == 'F':
                    if msg['from'] == 'Applet':
                        commsList[ARDUINO].write('F')
                    elif msg['from'] == 'Arduino':
                        commsList[APPLET].write('{"com":"statusUpdate", "status":"Finish Calibrate"}')

                elif com == 'C':
                    if msg['from'] == 'Applet':
                        commsList[ARDUINO].write('C')
                    elif msg['from'] == 'Arduino':
                        commsList[APPLET].write('{"com":"statusUpdate", "status":"Finish Calibrate"}')

                elif com == 'Q':
                    if msg['from'] == 'Applet':
                        commsList[ARDUINO].write('Q')
                    elif msg['from'] == 'Arduino':
                        commsList[APPLET].write('{"com":"statusUpdate", "status":"Finish Calibrate (Right-facing)"}')

                elif com == 'RST':
                    exploring = False

                ## G, H: EX -> FP transition (from Applet)
                elif com == 'G':
                    exploring = False
                    commsList[ARDUINO].write('G')
                    commsList[APPLET].write('{"com":"statusUpdate", "status":"calibrated for FP"}')
                    commsList[ANDROID].write(';{"com":"statusUpdate", "status":"Exploration Complete"}')

                    jsonExpHex = " Exp " + expHex
                    jsonObsHex = ", Obj " + obsHex
                    jsonImgs = ", Img " + str(imgs)

                    ## We try this - send at once
                    jsonString = jsonExpHex + jsonObsHex + jsonImgs
                    data = {'com': 'statusUpdate', 'status': jsonString}
                    commsList[ANDROID].write(json.dumps(data))

                elif com == 'H':
                    exploring = False
                    commsList[ARDUINO].write('H')
                    commsList[APPLET].write('{"com":"statusUpdate", "status":"calibrated for FP"}')
                    commsList[ANDROID].write(';{"com":"statusUpdate", "status":"Exploration Complete"}')

                    jsonExpHex = " Exp " + expHex
                    jsonObsHex = ", Obj " + obsHex
                    jsonImgs = ", Img " + str(imgs)

                    ## We try this - send at once
                    jsonString = jsonExpHex + jsonObsHex + jsonImgs
                    data = {'com': 'statusUpdate', 'status': jsonString}
                    commsList[ANDROID].write(json.dumps(data))

                ## MDF: From Applet to Android
                elif com == 'MDF':
                    commsList[ANDROID].write(json.dumps(msg))

                ## Android request for raw images, send them an array of JSON strings
                ## Update: Change to Applet (WiFi) - Bluetooth will have packet loss since they can receive in max 1KB packets
                elif msg['com'] == 'M':
                    correctImages = SendRawImages(correctImages)
                    data = {'com': 'Raw Image String', 'imgRaw': correctImages}
                    commsList[APPLET].write(json.dumps(data))

                elif msg['com'] == 'IR':
                    commsList[ANDROID].write(';{"com": "statusUpdate", "status": "Running Image Recognition"}')
                    commsList[APPLET].write('{"com": "statusUpdate", "status": "Running Image Recognition"}')

                elif msg['com'] == 'I':
                    rawCapture = PiRGBArray(camera) #, size=camera.resolution)
                    camera.capture(rawCapture, 'bgr')
                    image = rawCapture.array
                    img1 = image[120:, 0:139, :]
                    img2 = image[120:, 139:277, :]
                    img3 = image[120:, 277:416, :]

                    rect1, leftprediction = imageRec.predict(img1)
                    rect2, midprediction = imageRec.predict(img2)
                    rect3, rightprediction = imageRec.predict(img3)

                    rects = []
                    rects.append(rect1)
                    rects.append(rect2)
                    rects.append(rect3)

                    ids = []
                    ids.append(leftprediction)
                    ids.append(midprediction)
                    ids.append(rightprediction)

                    print(rects, ids)

                    for i, rect in enumerate(rects):
                        if rect is not None:
                            image = cv2.rectangle(image, (i * 120 + rect[0], 120 + rect[1]), (i * 120 + rect[2], 120 + rect[3]), (0,255,0), 2)
                        if ids[i] > 0: # Don't save those that default return 0
                            if ids[i] not in correctIdx: # Following Algo - save first instance instead of overwriting
                                cv2.imwrite(os.path.join(imagedir, str(ids[i]) + '.jpg'), image)
                                correctIdx.append(ids[i])
                                print("Image saved: " + str(ids[i]) + '.jpg')
                    # index = index + 1
                    data = {'com':'Image Taken', 'left': leftprediction,'middle': midprediction, 'right':rightprediction}
                    commsList[APPLET].write(json.dumps(data))
                    print('Left Prediction: ', leftprediction)
                    print('Middle Prediction: ', midprediction)
                    print('Right Prediction: ', rightprediction)

    except Exception as e:
        print("[MAIN_ERROR] Error. Prepare to shutdown...")

    finally:
        commsList[ARDUINO].disconnect()
        commsList[ANDROID].disconnect()
        commsList[APPLET].disconnect()
        camera.stop_preview()
        camera.close()
        logfile.close()
        sys.exit(0)