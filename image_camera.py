# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (390, 240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(390, 240))

# allow the camera to warmup
time.sleep(0.1)

camera.capture(rawCapture, format="bgr")
image = rawCapture.array
filename = 'image0000.jpg'

cv2.imwrite(filename, image)
print("Saved " + filename)