from multiprocessing import Process, Queue
from time import sleep

from ArduinoComms import ArduinoComm
from AndroidComms import AndroidComm
from AppletComms import AppletComm

import numpy as np
import os
import json
import sys
from datetime import datetime
import argparse

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

    ## Initialise variables
    running = True
    exploring = False
    obsHex = ''
    expHex = ''
    imgs = ''

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
                    ## All these is senior's group formatting JSON string in RPi.
                    ## We decided to bypass that since Applet can read the message by itself.
                    # Received Sensor Data
                    # fl = msg["fl"]
                    # fm = msg["fm"]
                    # fr = msg["fr"]
                    # rf = msg["rf"]
                    # rb = msg["rb"]
                    # left = msg["left"]
                    # data = {'fl': fl, 'fm': fm, 'fr': fr, 'rt': rf, 'rb': rb, 'left': left}
                    # commsList[APPLET].write(json.dumps(data))
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
                    # data = {'com':'statusUpdate', 'status':jsonExpHex}
                    # commsList[ANDROID].write(json.dumps(data))
                    # #commsList[ANDROID].write(';' + json.dumps(data))
                    jsonObsHex = ", Obj " + obsHex
                    # data = {'com':'statusUpdate', 'status':jsonObsHex}
                    # commsList[ANDROID].write(json.dumps(data))
                    # #commsList[ANDROID].write(';' + json.dumps(data))
                    jsonImgs = ", Img " + str(imgs)
                    # data = {'com':'statusUpdate', 'status':jsonImgs}
                    # commsList[ANDROID].write(json.dumps(data))
                    # #commsList[ANDROID].write(';' + json.dumps(data))

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
                    # data = {'com':'statusUpdate', 'status':jsonExpHex}
                    # commsList[ANDROID].write(json.dumps(data))
                    # #commsList[ANDROID].write(';' + json.dumps(data))
                    jsonObsHex = ", Obj " + obsHex
                    # data = {'com':'statusUpdate', 'status':jsonObsHex}
                    # commsList[ANDROID].write(json.dumps(data))
                    # #commsList[ANDROID].write(';' + json.dumps(data))
                    jsonImgs = ", Img " + str(imgs)
                    # data = {'com':'statusUpdate', 'status':jsonImgs}
                    # commsList[ANDROID].write(json.dumps(data))
                    # #commsList[ANDROID].write(';' + json.dumps(data))

                    ## We try this - send at once
                    jsonString = jsonExpHex + jsonObsHex + jsonImgs
                    data = {'com': 'statusUpdate', 'status': jsonString}
                    commsList[ANDROID].write(json.dumps(data))

                ## MDF: From Applet to Android
                elif com == 'MDF':
                    ## All these is senior's group formatting JSON string in RPi.
                    ## We decided to bypass that since Android can read the message by itself.
                    # Received MDF from applet, relay it to Android
                    # expHex = msg['expMDF']
                    # obsHex = msg['objMDF']  # obj and obs used interchangeably wah toh
                    # robotPos = msg['pos']
                    # imgs = msg['imgs']
                    #
                    # data = {'mapState': {'com': 'GS', 'obstacles': obsHex,
                    #                      'explored': expHex, 'robotPosition': robotPos, 'imgs': imgs}}

                    ## If line below is not a needed visual, we can leave it out
                    # commsList[ANDROID].write(
                    #     ';{"com": "statusUpdate", "status": "Moving to (' + str(robotPos[0]) + ',' + str(
                    #         robotPos[1]) + ')"}')
                    # commsList[ANDROID].write(';' + json.dumps(data))

                    commsList[ANDROID].write(json.dumps(msg))

    except Exception as e:
        print("[MAIN_ERROR] Error. Prepare to shutdown...")

    finally:
        commsList[ARDUINO].disconnect()
        commsList[ANDROID].disconnect()
        commsList[APPLET].disconnect()
        logfile.close()
        sys.exit(0)