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

def z_brightness(frame): # use this to get average brightness of each frame
    norm_frame = frame/np.max(frame)
    bright_avg = np.average(norm_frame)
    return bright_avg

def webcam_LK():
    web_lk_pub = rospy.Publisher('web_lk_topic', Float32) # ros topic lk init
    web_z_pub = rospy.Publisher('web_z_topic', Float32) # ros topic z init
    rospy.init_node('mcp_web_node', anonymous=True) # ros node init
    rate = rospy.Rate(FPS) # 10hz
    
    webcam = cv.VideoCapture(4) # usb logitech webcam
    if not (webcam.isOpened()):
        print("Could not open video device")
    
    webcam.set(cv.CAP_PROP_FRAME_WIDTH, DESIREDWIDTH) # 640
    webcam.set(cv.CAP_PROP_FRAME_HEIGHT, DESIREDHEIGHT) # 480
    webcam.set(cv.CAP_PROP_FPS, FPS)
    
    # take ref_frame:
    ret, ref_frame = webcam.read()
    ref_frame = webcam_process(ref_frame)
    
    # LK parameters: 
    feature_params = dict( maxCorners = 100, 
                            qualityLevel = 0.01, # between 0 and 1. Lower numbers = higher quality level. 
                            minDistance = 5.0, # distance in pixels between points being monitored. 
                            blockSize = 3,
                            useHarrisDetector = False, # Shi-Tomasi better for corner detection than Harris for fibrescope. 
                            k = 0.04 ) # something to do with area density, starts centrally. high values spread it out. low values keep it dense.
    lk_params = dict( winSize = (45, 45),
                maxLevel = 2,
                criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))

    color = np.random.randint(0, 255, (500, 3)) # Create some random colors 
    
    p0 = cv.goodFeaturesToTrack(ref_frame, mask = None, **feature_params) # Shi-Tomasi corner detection
    # p0 = cv.cornerHarris(ref_frame, 10,10,0.3) # Harris corner detection, ERROR. figure out how to use this!! 
    # cv.imshow('ref frame temp',ref_frame)
    mask_OF = np.zeros_like(ref_frame)

    p1,st,err = None,None,None
    
    while not rospy.is_shutdown():
        ret, frame = webcam.read()
        if not ret: break
        
        frame_filt = webcam_process(frame)
        cv.imshow("Webcam frame processed",frame_filt)
        
        p1,st,err = cv.calcOpticalFlowPyrLK(ref_frame, frame_filt, p0, p1, st, err,**lk_params)
        if p1 is not None:
            good_new = p1[st==1]
            good_old = p0[st==1]
        
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask_OF = cv.line(mask_OF, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            frame_filt = cv.circle(frame_filt, (int(a), int(b)), 5, color[i].tolist(), -1)
        img = cv.add(frame_filt, mask_OF)
        cv.imshow('Optical Flow - Lucas-Kanade', img)
        
        # publish value
        web_lk_val = np.mean(p1[...,0])
        # rospy.loginfo(web_lk_val)
        web_lk_pub.publish(web_lk_val)
        web_z_val = (z_brightness(frame_filt))
        # rospy.loginfo(web_z_val)
        web_z_pub.publish(web_z_val)
        
        rate.sleep()
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            print('Quitting...')
            break
    webcam.release()
    cv.destroyAllWindows()
    
if __name__ == '__main__':
    try:
        webcam_LK()
    except rospy.ROSInterruptException:
        cv.destroyAllWindows()
        pass

