#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Detection on flash light on ground from hexacopter

written by Matthieu Magnon
2016-2017
Dassault UAV Challenge
"""


#import libs
import cv2
import numpy as np
import time

#begin streaming
cap = cv2.VideoCapture(0)
cv2.startWindowThread()
cv2.namedWindow("imgFPV", cv2.WINDOW_AUTOSIZE)

frame = cv2.imread('light.jpg') #2 simple, 4 bruit, 5 dir2

cv2.imshow("imgFPV", frame)

#convert frame to monochrome and blur
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (9,9), 0)

#use function to identify threshold intensities and locations
(minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(blur)

#threshold the blurred frame accordingly
hi, threshold = cv2.threshold(blur, maxVal-20, 230, cv2.THRESH_BINARY)
thr = threshold.copy()
cv2.imwrite('light_mask_2.jpg',threshold)

#resize frame for ease
cv2.resize(thr, (300,300))

edged = cv2.Canny(threshold, 50, 150)
contours, hierarchy = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#print(len(lightcontours))

######################
areaArray = []
#contours, _ = cv2.findContours(ball_ycrcb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for i, c in enumerate(contours):
    area = cv2.contourArea(c)
    areaArray.append(area)

#first sort the array by area
sorteddata = sorted(zip(areaArray, contours), key=lambda x: x[0], reverse=True)

#find the nth largest contour [n-1][1], in this case 2
secondlargestcontour = sorteddata[0][1]


#draw it
x, y, w, h = cv2.boundingRect(secondlargestcontour)
cv2.drawContours(frame, secondlargestcontour, -1, (255, 0, 0), 2)
cv2.rectangle(frame, (x, y), (x+w, y+h), (0,255,0), 2)
cv2.imwrite('output3.jpg', frame)
