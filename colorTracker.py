#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */

import controller_include as ci
import csv


import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

import actuator_sim as ser

dd = diff_drive
#ref = dd.H_REF()
tim = dd.H_TIME()

#ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
#ROBOT_CHAN_VIEW   = 'robot-vid-chan'
#ROBOT_TIME_CHAN  = 'robot-time'

CONTROLLER_REF_NAME  = 'controller-ref-chan'

# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 640
ny = 480

#r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
#r.flush()
#v = ach.Channel(ROBOT_CHAN_VIEW)
#v.flush()
#t = ach.Channel(ROBOT_TIME_CHAN)
#t.flush()


e = ach.Channel(ci.CONTROLLER_REF_NAME)
e.flush()
controller = ci.CONTROLLER_REF()

i=0
Ts = .1



cap = cv2.VideoCapture(1)

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    
    ret, frame = cap.read()
    #img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    img = frame
    #hight, width, depth = img.shape
    #print "size", hight, width, depth

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # img        = cv image in BGR format
    
    controller.width = nx
    
    # Convert RGB to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Define upper and lower range of green color in HSV
    lower_blue = np.array([0,100,100], dtype=np.uint8)
    upper_blue = np.array([20,255,255], dtype=np.uint8)

    # Threshold the HSV image to get only green colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    
    kernel = np.ones((5,5), np.uint8)
    erosion = cv2.erode(mask, kernel, iterations = 5)
    dilation = cv2.dilate(erosion, kernel, iterations = 5)

    # Use findContours to get the boundry of the green blob
    contours,hierarchy = cv2.findContours(dilation,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # Look through all the seperate contours and highlight the boundry and centroid
    for cnt in contours:
		# Calculate moments
        moments = cv2.moments(cnt)   
        # maximum error:   
        controller.error = nx                    
        if moments['m00']!=0:
            x = int(moments['m10']/moments['m00'])
            y = int(moments['m01']/moments['m00'])
            print 'Center of Mass = ', '(', x, ', ', y, ')'
            
            # get the error from the center width:
            controller.x = x - (nx/2)
            controller.y = y - (ny/2)
            
            # draw contours 
            cv2.drawContours(img,[cnt],0,(0,0,255),1)   
            # draw centroids in red
            cv2.circle(img,(x,y),10,(0,0,255),-1)      

    cv2.imshow('wctrl',img)
    
    cv2.waitKey(10)
    
    # send the error to controller
    e.put(controller)

   
   
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
