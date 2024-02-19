#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int32 # , Float64MultiArray, Float32MultiArray
# from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2 as cv # pip install opencv-python
from pypylon import pylon # cd pypylon > pip install .
from pypylon import genicam

pressure_snsr_val = 0.0
pump_state = 0.0

# define constant parameters - in CAPS
FPS = 10.0 # 10 fps
# TOT_FRAMES = int(FPS*60) # 10 fps, 60 secs (1 min) long recording
# TOT_FRAMES = int(FPS*5) # 5 secs
DESIREDWIDTH = 640
DESIREDHEIGHT = 480

# fibrescope image enhancement parameters: 
CONTRAST = 3
BRIGHTNESS = 5

def fibrescope_process(frame):

    kernel = np.ones((2,2),np.uint8)
    gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
    mask_blank = np.zeros_like(gray,dtype='uint8') # ,dtype='uint8'
    # x,y,w,h = 350,280,200,110 # after resizing frame size. 
    # rect = cv.rectangle(mask_blank, (x, y), (x+w, y+h), (255,255,255), -1) # mask apply
    circle = cv.circle(mask_blank, (355,345), 100, (255,255,255), -1)
    masked = cv.bitwise_and(gray,gray,mask=circle)
    brightened = cv.addWeighted(masked, CONTRAST, np.zeros(masked.shape, masked.dtype), 0, BRIGHTNESS)     
    # binary = cv.threshold(brightened,57,255,cv.THRESH_BINARY)[1] # might remove: + cv.thresh_otsu
    # morph_open = cv.morphologyEx(binary,cv.MORPH_OPEN,kernel)
    # morph_close = cv.morphologyEx(morph_open,cv.MORPH_CLOSE,kernel)
    # dilated = cv.dilate(morph_close,kernel)

    return brightened

def z_brightness(frame): # use this to get average brightness of each frame
    norm_frame = frame/np.max(frame)
    bright_avg = np.average(norm_frame)
    return bright_avg

def fibrescope_execute():
    print("Fibrescope execution selected.")
    
    tlf = pylon.TlFactory.GetInstance()
    # t1 = tlf.CreateTl('BaslerGigE')
    fib_info = pylon.DeviceInfo()
    # fib_info = t1.CreateDeviceInfo()
    fib_ip = '192.168.0.2'
    fib_info.SetIpAddress(fib_ip) # might need to set temp address in ip config for pylon cam
    print("INFO-fib ", fib_info)
    fibrescope = pylon.InstantCamera(tlf.CreateDevice(fib_info)) # ERROR HERE
    print("Using device ", fibrescope.GetDeviceInfo().GetModelName())
    fibrescope.Open() 
    
    # set some basic parameters
    # fibrescope.TLParamsLocked = False # grab lock
    # fibrescope.Width.SetValue(DESIREDWIDTH) # 640
    # fibrescope.Height.SetValue(DESIREDHEIGHT) # 480
    fibrescope.AcquisitionFrameRateEnable.SetValue(True)
    fibrescope.AcquisitionFrameRateAbs.SetValue(FPS)
    fibrescope.GainAuto.SetValue('Off')
    fibrescope.GainRaw.SetValue(200)
    fibrescope.ExposureAuto.SetValue('Off')
    fibrescope.ExposureTimeAbs = 10000.0
    fibrescope.AcquisitionMode.SetValue("Continuous")
    fibrescope.PixelFormat = "Mono8"
    
    # fibrescope.TLParamsLocked = True # grab unlock
    fibrescope.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
    
    # take ref_frame: 
    ref_frame = fibrescope.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
    ref_frame = ref_frame.Array
    ref_frame = cv.resize(ref_frame,(DESIREDWIDTH,DESIREDHEIGHT))
    if not ref_frame.GrabSucceeded(): print("ERROR: Unable to grab REFERENCE frame.")
    else: cv.imshow("Reference frame", ref_frame)
    
    # lk-params load: 
    feature_params = dict( maxCorners = 100, 
                            qualityLevel = 0.01, # between 0 and 1. Lower numbers = higher quality level. 
                            minDistance = 5.0, # distance in pixels between points being monitored. 
                            blockSize = 3,
                            useHarrisDetector = False, # Shi-Tomasi better for corner detection than Harris for fibrescope. 
                            k = 0.04 ) # something to do with area density, starts centrally. high values spread it out. low values keep it dense.
    lk_params = dict( winSize = (45, 45),
                    maxLevel = 2,
                    criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    while not rospy.is_shutdown():
        frame = fibrescope.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        
        if not frame.GrabSucceeded(): 
            print("ERROR: Unable to grab frame.")
            break
        # bgr_img = converter.Convert(frame)
        # bgr_img = bgr_img.GetArray()
        bgr_img = frame.Array

        bgr_img = cv.resize(bgr_img,(DESIREDWIDTH,DESIREDHEIGHT)) # resize, not ideal method but seems like the only way left...
        
        cv.imshow("Fibrescope recording", bgr_img)
        
        frame_filt = fibrescope_process(bgr_img) # was: (cap,frame)
        # cv.imshow('FILTERED + CROPPED',frame_filt)
        
        # p1,st,err = cv.calcOpticalFlowPyrLK(ref_frame, frame_filt, p0, None, None, None,**lk_params)
        p1,st,err = cv.calcOpticalFlowPyrLK(ref_frame, frame_filt, p0, p1, st, err,**lk_params)
        # z_val = z_brightness(frame_filt)
        # magnitude, angle = cv.cartToPolar(p1[..., 0], p1[..., 1])
        if p1 is not None:
            good_new = p1[st==1]
            good_old = p0[st==1]
        
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask_OF = cv.line(mask_OF, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            frame = cv.circle(frame_filt, (int(a), int(b)), 5, color[i].tolist(), -1)
        img = cv.add(frame_filt, mask_OF)
        cv.imshow('Optical Flow - Lucas-Kanade', img)

        # publish value
        lk_val = np.mean(p1[...,0])
        rospy.loginfo(lk_val)
        lk_pub.publish(lk_val)
        if cv.waitKey(10) & 0xFF == ord('q'):
            print('Quitting...')
            break
        
    fibrescope.StopGrabbing()
    fibrescope.Close()
    cv.destroyAllWindows()
    
def OF_LK(cap,ref_frame,img_process,savefilename): # Lucas-Kanade, sparse optical flow, local solution
    lk_pub = rospy.Publisher('lk_topic', Float32)
    # LK OF parameters: 
    if img_process == webcam_process:
        print('LK: Webcam')
        feature_params = dict( maxCorners = 700, 
                                qualityLevel = 0.15, # between 0 and 1. Lower numbers = higher quality level. 
                                minDistance = 25.0, # distance in pixels between points being monitored. 
                                blockSize = 5,
                                useHarrisDetector = False, 
                                k = 0.04 ) # something to do with area density, starts centrally. high values spread it out. low values keep it dense. 
        lk_params = dict( winSize  = (45, 45),
                    maxLevel = 2,
                    criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    
    elif img_process == fibrescope_process:
        print('LK: Fibrescope')
        feature_params = dict( maxCorners = 100, 
                                qualityLevel = 0.01, # between 0 and 1. Lower numbers = higher quality level. 
                                minDistance = 5.0, # distance in pixels between points being monitored. 
                                blockSize = 3,
                                useHarrisDetector = False, # Shi-Tomasi better for corner detection than Harris for fibrescope. 
                                k = 0.04 ) # something to do with area density, starts centrally. high values spread it out. low values keep it dense.
        lk_params = dict( winSize = (45, 45),
                    maxLevel = 2,
                    criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    
    else:
        print("ERROR: Please enter a valid argument for imaging method used.")
        exit()
        
    # Parameters for lucas kanade optical flow
    # lk_params = dict( winSize  = (45, 45),
    #                 maxLevel = 2,
    #                 criteria = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 0.03))
    
    color = np.random.randint(0, 255, (500, 3)) # Create some random colors 
    
    p0 = cv.goodFeaturesToTrack(ref_frame, mask = None, **feature_params) # Shi-Tomasi corner detection
    # p0 = cv.cornerHarris(ref_frame, 10,10,0.3) # Harris corner detection, ERROR. figure out how to use this!! 
    # cv.imshow('ref frame temp',ref_frame)
    mask_OF = np.zeros_like(ref_frame)

    p1,st,err = None,None,None
    while not rospy.is_shutdown():
        
        ret, frame = cap.read()
        if not ret: break 

        frame_filt = img_process(frame) # was: (cap,frame)
        # cv.imshow('FILTERED + CROPPED',frame_filt)
        
        # p1,st,err = cv.calcOpticalFlowPyrLK(ref_frame, frame_filt, p0, None, None, None,**lk_params)
        p1,st,err = cv.calcOpticalFlowPyrLK(ref_frame, frame_filt, p0, p1, st, err,**lk_params)
        # z_val = z_brightness(frame_filt)
        # magnitude, angle = cv.cartToPolar(p1[..., 0], p1[..., 1])
        if p1 is not None:
            good_new = p1[st==1]
            good_old = p0[st==1]
        
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask_OF = cv.line(mask_OF, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            frame = cv.circle(frame_filt, (int(a), int(b)), 5, color[i].tolist(), -1)
        img = cv.add(frame_filt, mask_OF)
        cv.imshow('Optical Flow - Lucas-Kanade', img)

        # publish value
        lk_val = np.mean(p1[...,0])
        rospy.loginfo(lk_val)
        lk_pub.publish(lk_val)
    
        if cv.waitKey(10) & 0xFF == ord('q'):
            print('Quitting...')
            break
        
    
    
def publisher():
    fib_lk_pub = rospy.Publisher('fib_lk', Float32) # queue_size=10, only for ROS hydro, new feature
    pump_state_pub = rospy.Publisher('pump_state', Float32)
    pressure_val_pub = rospy.publisher('pressure_val', Float32)
    rospy.init_node('mcp_data_fib_lk', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        fib_lk_val = ...
        pump_state_val = ...
        pressure_val = ...
        rospy.loginfo(fib_lk_val)
        rospy.loginfo(pump_state_val)
        rospy.loginfo(pressure_val)
        fib_lk_pub.publish(fib_lk_val)
        pump_state_pub.publish(pump_state_val)
        pressure_val_pub.publish(pressure_val)
        
        rate.sleep()

def main():

    # get ref frame: 
    while not rospy.is_shutdown():
        ref_frame = fibrescope_execute()
    
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass