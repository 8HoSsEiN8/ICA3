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
import controller_include2 as ci2
import collections

import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import math

import actuator_sim as ser

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'

CONTROLLER_REF_NAME  = 'controller-ref-chan'

# CV setup 
cv.NamedWindow("wctrl", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))



e = ach.Channel(ci.CONTROLLER_REF_NAME)
e.flush()
controller = ci.CONTROLLER_REF()

e2 = ach.Channel(ci2.DYNO_REF_NAME)
e2.flush()
controller2 = ci2.DYNO_REF()

i=0

errorWindow = collections.deque(maxlen = 25)
errorWindow.append(0)
Kp = 1
Ki = 1
oldError = 0
Kd = .2

width = 640
hight = 480

print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
	[statuss, framesizes] = e.get(controller, wait=True, last=True)
	x = controller.x
	y = controller.y
	maxError = width/2.0
	print 'x, y', x, ' &', y
	
	k = .001
	controller2.mot1 = k * x
	controller2.mot2 = k * y
	e2.put(controller2)
	
	'''
	# normalize error so that +/- 320 error is +/-1
	errorNorm = abs(error) / maxError
	
	# Proportional Controller:
	Sp = errorNorm * Kp
	# Integral Controller:
	errorWindow.append(errorNorm)
	eSum = sum(errorWindow)
	Si = (eSum * .1) * Ki
	# Derivative Controller:
	Sd = ( (errorNorm - oldError) / .1 ) * Kd
	oldError = errorNorm
	
	# sum them up and if more than 1, set to 1
	Speed = Sp + Si + Sd
	if (Speed > 1):
		Speed = 1
		
	print 'Speed = ', Speed
	
	if ( -maxError <= error <= 0):		# Object is left of the Center
		print 'Object is left of the Center'
		ref.ref[0] = Speed
		ref.ref[1] = -Speed
	elif ( 0 <= error <= maxError ):	# Object is right of the Center
		print 'Object is right of the Center'		
		ref.ref[0] = -Speed
		ref.ref[1] = Speed
	else:								# No Object, Searching for it
		print 'No Object, Searching for it'
		ref.ref[0] = -.5
		ref.ref[1] = .5
	
	# control the robot	
	r.put(ref);
	'''
#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
