# File:     main.py
#
# Author:   Liuchuyao Xu, 2019

import cv2
import serial

with serial.Serial("COM3") as ser:
    while True:
        c = ser.read(1)
        if c == 'b\'x\'':
            cv2.namedWindow("preview")
            camera = cv2.VideoCapture(0)
            if camera.isOpened():
                result, frame = camera.read()
                cv2.imwrite("image.png", frame)

