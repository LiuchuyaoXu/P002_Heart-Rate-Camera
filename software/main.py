# File:     main.py
#
# Author:   Liuchuyao Xu, 2019

import cv2
import serial

# with serial.Serial("COM5") as ser:
#     while(True):
#         print(ser.read())

cv2.namedWindow("preview")
camera = cv2.VideoCapture(0)
if camera.isOpened():
    result, frame = camera.read()
    cv2.imwrite("image.png", frame)
