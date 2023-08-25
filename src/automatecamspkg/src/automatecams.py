#!/usr/bin/env python3
#-*- coding: utf-8 -*-
# import sys
# sys.path.append('/usr/local/lib/python3.7/site-packages')
import time
import multiprocessing as mp
import rospy
from std_msgs.msg import Float32MultiArray 
import numpy as np
import cv2 as cv
# from pypylon_opencv_viewer import BaslerOpenCVViewer # might be unecessary
from pypylon import pylon
# print('imports done successfully!')
class automation():
    def __init__(self):
        self.tot_frames = 30*2*60 # 30 fps, 2 mins.
        self.fps = 30 # 30 fps
        print('init executed ---------------------------')
 
    def webcam_execute(self):
        print("Webcam execution selected.")
        webcam = cv.VideoCapture(2) # usb webcam
        if not (webcam.isOpened()):
            print("Could not open video device")
        webcam.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        webcam.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        # webcam.set(cv.CAP_PROP_FPS, self.fps)
        webcam.set(cv.CAP_PROP_FPS, 30)
        # webcam_writer = cv.VideoWriter('webcam2/webcam*.mp4',cv.VideoWriter_fourcc(*'mp4v'),self.fps,(640,480))
        webcam_writer = cv.VideoWriter('webcam2/webcam*.mp4',cv.VideoWriter_fourcc(*'mp4v'),30,(640,480))

        # for counter in range(self.tot_frames): 
        for counter in range(30*2*60):
            ret, frame = webcam.read()
            if not ret: break 
            webcam_writer.write(frame)
            cv.imshow('Webcam Binary',frame)

            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break    
        webcam.release()                                    
        webcam_writer.release()
        cv.destroyAllWindows()

    def fibrescope_execute(self):
        print("Fibrescope execution selected")
        
        fibrescope = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        print("Using device ", fibrescope.GetDeviceInfo().GetModelName())
        fibrescope.Width.SetValue(640)
        fibrescope.Height.SetValue(480)
        # also need to set other setting values... Gain and exposure. 
        # !!!
        # others keep default
        fibrescope.Open() # Open the camera 
        fibrescope.AcquisitionMode.SetValue('SingleFrame')
        fibrescope.StartGrabbing(pylon.GrabStrategy_OneByOne)
        self.ref_frame = fibrescope.RetrieveResult()
        fibrescope.StopGrabbing()
        time.sleep(2)
        fibrescope.AcquisitionFrameRateAbs.SetValue(self.fps)
        fibre_writer = cv.VideoWriter('fibrescope2/videos/fibrescope*.mp4',cv.VideoWriter_fourcc(*'mp4v'),self.fps,(fibrescope.Width.GetValue(),fibrescope.Height.GetValue()))
        fibrescope.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        
        for counter in range(self.tot_frames):
            # do the thing
            ret, frame = fibrescope.RetrieveResult(5000, pylon.GrabStrategy_LatestImageOnly)
            if not ret: break
            frame = automation.filter_frame_fibrescope(frame) # bonarize frame
            fibre_writer.write(frame)
            cv.imshow('Fibrescope Binary',frame)
            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break
        
        fibrescope.StopGrabbing()
        fibre_writer.release()
        cv.destroyAllWindows()
        
    def robot_setup(self):
        print("Bringing end-effector to position for set-up.")
        pub = rospy.Publisher('joint_demand', Float32MultiArray, queue_size=10)
        rospy.init_node('Mover6Pub', anonymous=True)
        rate = rospy.Rate(1/3) # 1/3 hz
        while not rospy.is_shutdown():
            jnt_val = Float32MultiArray()
            jnt_val.data = [0,0,0,0,60,0] # home, wrist down
            pub.publish(jnt_val)
            rate.sleep()

            jnt_val.data = [0,0,0,0,0,0] # change to align axes 
            pub.publish(jnt_val)
            rate.sleep()
            print("End-effector alignment complete")
            break
        
        self.gripper_ready = input('Adjust gripper from rviz and PRESS Y/y when ready:')
    def robot_test(self): 
        print("This is case three: Test robot")
        pub = rospy.Publisher('joint_demand', Float32MultiArray, queue_size=10)
        rospy.init_node('Mover6Pub', anonymous=True)

        rate = rospy.Rate(1/3) # 1/3 hz
        while not rospy.is_shutdown():
            jnt_val = Float32MultiArray()

            if self.gripper_ready == 'Y' or 'y':
                # jnt_val.data = [0,0,0,0,np.deg2rad(84),0]
                jnt_val.data = [0,0,0,0,np.deg2rad(80),0] # move len/2
                pub.publish(jnt_val)
                rate.sleep()

                for i in range(10):
                    jnt_val.data = [0,np.deg2rad(15),np.deg2rad(40),0,np.deg2rad(30),0] # move -len
                    pub.publish(jnt_val)
                    rate.sleep()
                    jnt_val.data = [0,np.deg2rad(30),np.deg2rad(20),0,np.deg2rad(35),0] # move len
                    pub.publish(jnt_val)
                    rate.sleep()
                    print('Iteration: ',i+1)
                jnt_val.data = [0,0,0,0,0,0] # move -len/2
                print('Finished. i= ',i+1) # check for loop execution
                break
    def polaris_callback(data):
        rospy.loginfo(rospy.get_caller_id(), data.data)
        rospy.init_node()
    def polaris_sub():
        # polaris ros subscriber, use this to automate recording and saving. 
        print('Polaris system selected')
        rospy.init_node('polaris_tracker', anonymous=True)
        rospy.Subscriber('polaris_data', Float32MultiArray, automation.polaris_callback) # topic name = polaris_data - needs to be same in publisher too.
        rospy.spin()
    def wrong_input():
        print("This is the default case. Input not recognised, please try again.")
        
    
def main():
    switcher = {
        1: automation.webcam_execute,
        2: automation.fibrescope_execute,
        3: automation.polaris_sub,
        4: automation.robot_test,    
    }
    case_number = int(input('Enter input number: 1 - webcam; 2 - fibrescope; 3 - Test robot.'))
    # case_number = 1
    # get the function from switcher dictionary
    # if the case number is not found, default to case_default
    chosen_case = switcher.get(case_number, automation.wrong_input)

    # execute the chosen function
    chosen_case(())
    print('main executed -----------------------')

if __name__ == '__main__':
    main()
    print('main TO BE executed -----------------------')
    # use multiprocessing to run mcp tracker and polaris in parallel

    