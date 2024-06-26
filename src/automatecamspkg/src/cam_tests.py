#!/usr/bin/env python3

import cv2 as cv # pip install opencv-python
import os
import time
# import rospy
# from std_msgs.msg import String
from pypylon import pylon # cd pypylon > pip install .
from pypylon import genicam
# import torchvision # pip install torch 
# import torchvision.transforms


FPS = 20.0 # 20 fps
TOT_FRAMES = int(FPS*5) # 5 secs
DESIREDWIDTH = 640
DESIREDHEIGHT = 480

class test_cam():
    # def talker():
    #     pub = rospy.Publisher('chatter', String, queue_size=10)
    #     rospy.init_node('talker', anonymous=True)
    #     rate = rospy.Rate(10) # 10hz
    #     # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #         # rate.sleep()
    
    # def callback(data):
    #     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    # def listener(): 
    # # In ROS, nodes are uniquely named. If two nodes with the same
    # # name are launched, the previous one is kicked off. The
    # # anonymous=True flag means that rospy will choose a unique
    # # name for our 'listener' node so that multiple listeners can
    # # run simultaneously.
    #     rospy.init_node('listener', anonymous=True)

    #     rospy.Subscriber("chatter", String, test_cam.callback)
    # # spin() simply keeps python from exiting until this node is stopped
    #     # rospy.spin()
    def web_record():
        cap = cv.VideoCapture(0) # HPC = 0, laptop = 4/0
        
        if cap.isOpened():
            cap.set(cv.CAP_PROP_FRAME_WIDTH, DESIREDWIDTH) # 640
            cap.set(cv.CAP_PROP_FRAME_HEIGHT, DESIREDHEIGHT) # 480
            cap.set(cv.CAP_PROP_FPS, FPS) # 20.0
            
            root = os.path.join('src/automatecamspkg/src/outputs/webcam')
            filename = os.path.join('webcam-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.mp4')
            cam_writer = cv.VideoWriter(os.path.join(root,filename),cv.VideoWriter_fourcc(*'mp4v'),FPS,(DESIREDWIDTH,DESIREDHEIGHT),True)

            for _ in range(TOT_FRAMES):
                ret,frame = cap.read()
                if not ret: break
                
                # test_cam.talker()
                # test_cam.listener()
                cam_writer.write(frame)
                cv.imshow('cam_test',frame)
                
                if cv.waitKey(10) & 0xFF == ord('q'):
                    print('Quitting...')
                    break 
        else:
            print('ERROR: Unable to open camera')
        
        cam_writer.release()
        cap.release()
        cv.destroyAllWindows()
    # def format_ip_config(cfg_str):
    #     result = []
    #     cfg = int(cfg_str)
    #     if cfg & 1:
    #         result.append("PersistentIP")
    #     if cfg & 2:
    #         result.append("DHCP")
    #     if cfg & 4:
    #         result.append("LLA")
    #     return ", ".join(result)    
    def fibre_record():
        tlf = pylon.TlFactory.GetInstance()
        fib_info = pylon.DeviceInfo()
        fib_static_ip = '192.168.0.2'
        # fib_info.SetPropertyValue('IpAddress', fib_static_ip)
        fib_info.SetIpAddress(fib_static_ip)
        fibrescope = pylon.InstantCamera(tlf.CreateDevice(fib_info)) # ERROR HERE
        print("Using device ", fibrescope.GetDeviceInfo().GetModelName())
        fibrescope.Open() 
        
        # set some basic parameters
        # fibrescope.TLParamsLocked = False # grab lock
        # fibrescope.Width.SetValue(DESIREDWIDTH) # 640
        # fibrescope.Height.SetValue(DESIREDHEIGHT) # 480
        # Setting value for width and height crops the image :( 
        
        fibrescope.AcquisitionFrameRateEnable.SetValue(True)
        fibrescope.AcquisitionFrameRateAbs.SetValue(FPS)
        fibrescope.GainAuto.SetValue('Off')
        fibrescope.GainRaw.SetValue(200)
        fibrescope.ExposureAuto.SetValue('Off')
        fibrescope.ExposureTimeAbs = 10000.0
        fibrescope.AcquisitionMode.SetValue("Continuous")
        fibrescope.PixelFormat = "Mono8" # "YUV422Packed"
        
        fib_root = os.path.join('src/automatecamspkg/src/outputs/fibrescope')
        fib_filename = 'fibrescope-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.mp4'
        fib_writer = cv.VideoWriter(os.path.join(fib_root,fib_filename),cv.VideoWriter_fourcc(*'mp4v'),FPS,(DESIREDWIDTH,DESIREDHEIGHT),0)
        # fib_writer = cv.VideoWriter(os.path.join(fib_root,fib_filename),cv.VideoWriter_fourcc(*'XVID'),FPS,(DESIREDWIDTH,DESIREDHEIGHT),True,pixelformat="uyvy422")
        # fib_writer = FFMPEG_VideoWriter
        # fibrescope.TLParamsLocked = True # grab unlock
        fibrescope.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        
        # converter = pylon.ImageFormatConverter()
        # converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        # converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        curr_time = time.time()
        # transform = torchvision.transforms.Resize((DESIREDHEIGHT,DESIREDWIDTH))
        
        for counter in range(TOT_FRAMES):
            frame = fibrescope.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            
            if not frame.GrabSucceeded(): 
                print("ERROR: Unable to grab frame.")
                exit()
            # bgr_img = converter.Convert(frame)
            # bgr_img = bgr_img.GetArray()
            bgr_img = frame.Array
            
            bgr_img = cv.resize(bgr_img,(DESIREDWIDTH,DESIREDHEIGHT)) # resize, not ideal method but seems like the only way left...
            # bgr_img = transform(bgr_img)
            cv.imshow("Fibrescope recording", bgr_img)
            fib_writer.write(bgr_img)
            
            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break
            
            elapsed_time = time.gmtime(time.time() - curr_time)
            elapsed_time_format = "{:02d}:{:02d}".format(elapsed_time.tm_min,elapsed_time.tm_sec)
            print('Elapsed time: ',elapsed_time_format,' Frame: ',counter,'/',TOT_FRAMES)
            # time.sleep(1/FPS)
            
        fib_writer.release # inside or outside the loop, and why??
        fibrescope.StopGrabbing()
        fibrescope.Close()
        cv.destroyAllWindows()
def main():
    test_cam.web_record()
    
if __name__ == '__main__':
    main()
    print('Done')