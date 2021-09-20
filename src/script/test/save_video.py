#!/usr/bin/env python

import numpy as np
import cv2

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture('tcp://192.168.1.1:5555')

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'X264')
# out = cv2.VideoWriter('pyout1.h264', cv2.VideoWriter_fourcc(*'avc1'), 20, (640,360))
out =  cv2.VideoWriter('output.h264', fourcc, 15, (640,368),False)
# out = cv2.VideoWriter('output.h264',fourcc, 20.0, (640,360))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
        out.write(frame)

        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()