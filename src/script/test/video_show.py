#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
# cam = cv2.VideoCapture('tcp://192.168.1.1:5555')

cam = cv2.VideoCapture("output.264")
running = True
while running:
    # get current frame of video
    running, frame = cam.read()
    # print(frame)
    # print('\n\n')
    grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if running:
        # cv2.imshow('frame', frame)
        # cv2.imshow('video gray', grayFrame)
        cv2.imshow('video original', frame)
        if cv2.waitKey(1) & 0xFF == 27: 
            # escape key pressed
            running = False
    else:
        # error reading frame
        print 'error reading video feed'
cam.release()
cv2.destroyAllWindows()
