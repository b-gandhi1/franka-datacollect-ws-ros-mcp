#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int32 # , Float64MultiArray, Float32MultiArray
# from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2 as cv # pip install opencv-python

pressure_snsr_val = 0.0
pump_state = 0.0

# define constant parameters - in CAPS
FPS = 10.0 # 10 fps
# TOT_FRAMES = int(FPS*60) # 10 fps, 60 secs (1 min) long recording
# TOT_FRAMES = int(FPS*5) # 5 secs
DESIREDWIDTH = 640
DESIREDHEIGHT = 480

def webcam_process(frame):

    # width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    # height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    # frame = cv.resize(frame,(int(width/2),int(height/2)),)

    kernel = np.ones((4,4),np.uint8)
    gray = cv.cvtColor(frame,cv.COLOR_RGB2GRAY)
    mask_blank = np.zeros_like(gray,dtype='uint8') # ,dtype='uint8'
    x,y,w,h = 0,60,635,340 # (x,y) = top left params
    rect = cv.rectangle(mask_blank, (x, y), (x+w, y+h), (255,255,255), -1) # mask apply
    masked = cv.bitwise_and(gray,gray,mask=rect)
    # binary = cv.threshold(masked,50,255,cv.THRESH_BINARY)[1] 
    # morph_open = cv.morphologyEx(binary,cv.MORPH_OPEN,kernel)
    # morph_close = cv.morphologyEx(morph_open,cv.MORPH_CLOSE,kernel)
    # dilated = cv.dilate(morph_close,kernel)

    return masked 
def webcam_execute():
    print("Webcam execution selected.")
    
    webcam = cv.VideoCapture(4) # usb logitech webcam
    if not (webcam.isOpened()):
        print("Could not open video device")
    webcam.set(cv.CAP_PROP_FRAME_WIDTH, DESIREDWIDTH) # 640
    webcam.set(cv.CAP_PROP_FRAME_HEIGHT, DESIREDHEIGHT) # 480
    webcam.set(cv.CAP_PROP_FPS, FPS)
    
    