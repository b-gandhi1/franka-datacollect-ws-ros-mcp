#!/usr/bin/env python3
# above line is compulsary for ROS environments only. 
# -*- coding: utf-8 -*-
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
# from imageio import get_writer # video writer for pylon
import imageio.v3 as iio

print('imports done successfully!')
class automation():
    def __init__(self):
        # self.tot_frames = 30*2*60 # 30 fps, 2 mins.
        self.tot_frames = 30*5 # 5 secs
        self.fps = 30 # 30 fps
        self.desiredwidth = 640
        self.desiredheight = 480
        print('init executed ---------------------------')

    def arduino_pressure_callback(self,data): # pressure value check
        rospy.loginfo(rospy.get_caller_id() + "Pressure value %s", data.data)
        # self.pressure_snsr_val = data

    def arduino_pumpstatus_callback(self,data): # pump status check
        rospy.loginfo(rospy.get_caller_id() + "Pump status %s", data.data)
        # self.pump_state = data

    def arduino_sub(self): # subscriber for arduino pressure sensor
        rospy.init_node('arduino_vals', anonymous=True)
        self.pressure_snsr_val = rospy.Subscriber("pressure_val", Float32MultiArray, automation.arduino_pressure_callback)
        self.pump_state = rospy.Subscriber("pump_state", Float32MultiArray, automation.arduino_pumpstatus_callback)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def webcam_execute(self):
        print("Webcam execution selected.")
        webcam = cv.VideoCapture(4) # usb webcam
        if not (webcam.isOpened()):
            print("Could not open video device")
        webcam.set(cv.CAP_PROP_FRAME_WIDTH, self.desiredwidth) # 640
        webcam.set(cv.CAP_PROP_FRAME_HEIGHT, self.desiredheight) # 480
        # webcam.set(cv.CAP_PROP_FPS, self.fps)
        webcam.set(cv.CAP_PROP_FPS, self.fps)
        webcam_writer = cv.VideoWriter('src/automatecamspkg/src/outputs/webcam/webcam.mp4',cv.VideoWriter_fourcc(*'mp4v'),self.fps,(self.desiredwidth,self.desiredheight),True)
        
        timestamp = []
        pressure = []
        polaris = []
        frankajnt = []

        # for counter in range(self.tot_frames): 
        for counter in range(self.tot_frames):
            ret, frame = webcam.read()
            if not ret: break 
            webcam_writer.write(frame)
            cv.imshow('Webcam recording',frame)
            
            timestamp.append(time.time())
            # pressure_val = np.array(self.pressure_snsr_val,self.pump_state)
            # pressure.append(pressure_val)
            # polaris.append(self.polaris_pos)
            # frankajnt.append(self.franka_pos)

            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break    
        webcam.release()                                    
        webcam_writer.release()
        cv.destroyAllWindows()

        # save timestamps as csv 
        # header = ['Counter','Timestamp','Pressure (kPa)','Polaris Tx','Polaris Ty','Polaris Tz','Polaris Error','Polaris Q0','Polaris Qx','Polaris Qy','Polaris Qz','Franka EE','','','','','','','']
        # np.savetxt('src/outputs/webcam/timestamp*.csv', np.array(counter, timestamp, pressure, polaris, frankajnt), header=header, delimiter=',')
        
    def fibrescope_execute(self):
        print("Fibrescope execution selected")
        
        fibrescope = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        print("Using device ", fibrescope.GetDeviceInfo().GetModelName())
        fibrescope.Open() 
        # set some basic parameters
        fibrescope.Width.SetValue(self.desiredwidth) # 640
        fibrescope.Height.SetValue(self.desiredheight) # 480
        fibrescope.AcquisitionFrameRateAbs.SetValue(self.fps)
        fibrescope.GainAuto.SetValue('Off')
        fibrescope.GainRaw.SetValue(200)
        fibrescope.ExposureTimeAbs = 90000
        # fibrescope.UserSetSelector = "UserSetMCP"
        # fibrescope.UserSetSave.Execute()
        # fibrescope.UserSetDefaultSelector = "UserSetMCP"
        
        # also need to set other setting values... Gain and exposure. 
        # !!!
        # others keep default
        
        # fibre_writer = cv.VideoWriter('src/automatecamspkg/src/outputs/fibrescope/fibrescope.mp4',cv.VideoWriter_fourcc(*'mp4v'),self.fps,(self.desiredwidth,self.desiredheight),True)
        fibre_writer = iio.get_writer('src/automatecamspkg/src/outputs/fibrescope/fibrescope.mp4', fps=self.fps)
        fibrescope.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        timestamp = []
        pressure = []
        polaris = []
        frankajnt = []

        for counter in range(self.tot_frames):
            # if not ret: break
            # do the thing
            frame = fibrescope.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if frame.GrabSucceeded():
                img = frame.Array
                cv.imshow('Fibrescope recording',img)
                # fibre_writer.write(img) # saving video
                im = iio.imread(img) # imageio
                im.reshape((self.desiredheight,self.desiredwidth)) # imageio, needs resizing, width height order inverted. 
                fibre_writer.append_data(im) # imageio
                
                timestamp.append(time.time())
                # pressure_val = np.array(self.pressure_snsr_val,self.pump_state)
                # pressure.append(pressure_val)
                # polaris.append(self.polaris_pos)
                # frankajnt.append(self.franka_pos)

            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break
        # fibre_writer.release()
        fibre_writer.close() # imageio 
        fibrescope.StopGrabbing()
        fibrescope.Close()
        cv.destroyAllWindows()
        
        # save timestamps as csv
        # header = ['Counter','Timestamp','Pressure (kPa)','Polaris Tx','Polaris Ty','Polaris Tz','Polaris Error','Polaris Q0','Polaris Qx','Polaris Qy','Polaris Qz','Franka EE','','','','','','','']
        # np.savetxt('src/outputs/fibrescope/timestamp*.csv', np.array(counter, timestamp, pressure, polaris, frankajnt),header=header,delimiter=',')

    def franka_callback(self,data):
        rospy.loginfo(rospy.get_caller_id(), "Franka End Effector Position: %s", data.data)
        # self.franka_pos = data # ground truth

    def franka_sub(self): # subscriber for franka emika robot arm
        # subscribe to topic from ros2, use bridge. 
        rospy.init_node('frankaemika', anonymous=True)
        self.franka_pos = rospy.Subscriber('franka_ee_pos', Float32MultiArray, automation.franka_callback)
        rospy.spin()

    def polaris_callback(self,data):
        rospy.loginfo(rospy.get_caller_id(), data.data)
        # self.polaris_pos = data

    def polaris_sub(self): # state of the art comparison, also ground truth backup. 
        rospy.init_node('polaris', anonymous=True)
        rospy.init_node('polaris_tracker', anonymous=True)
        self.polaris_pos = rospy.Subscriber('polaris_data', Float32MultiArray, automation.polaris_callback) # topic name = polaris_data - needs to be same in publisher too.
        
        rospy.spin()
        
    def wrong_input():
        print("This is the default case. Input not recognised, please try again.")
        
    
def main():
    switcher = {
        1: automation.webcam_execute,
        2: automation.fibrescope_execute,
        # 3: automation.polaris_sub,
    }
    case_number = int(input('Enter input number: 1 - webcam; 2 - fibrescope.'))
    # case_number = 1
    # get the function from switcher dictionary
    # if the case number is not found, default to case_default
    chosen_case = switcher.get(case_number, automation.wrong_input)

    chosen_case(automation())

    # # begin multi process to run chosen_case and polaris together: 
    # polaris_process = mp.Process(target=automation.polaris_sub)
    # cam_process = mp.Process(target=chosen_case)
    # try: 
    #     # start multi process: 
    #     polaris_process.start()
    #     cam_process.start()

    #     # finish multi process: 
    #     polaris_process.join()
    #     cam_process.join()

    #     print('Multi-processes finished.')
    # except KeyboardInterrupt:
    #     print('*****ERROR: Manually interrupted*****')
    #     pass
    print('main executed -----------------------')

if __name__ == '__main__':
    main()
    automation.polaris_sub()
    # automation.arduino_sub()
    automation.franka_sub()
    print('main TO BE executed -----------------------')

    