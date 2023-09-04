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
# print('imports done successfully!')
class automation():
    def __init__(self):
        self.tot_frames = 30*2*60 # 30 fps, 2 mins.
        self.fps = 30 # 30 fps
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
        webcam = cv.VideoCapture(2) # usb webcam
        if not (webcam.isOpened()):
            print("Could not open video device")
        webcam.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        webcam.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        # webcam.set(cv.CAP_PROP_FPS, self.fps)
        webcam.set(cv.CAP_PROP_FPS, 30)
        # webcam_writer = cv.VideoWriter('webcam2/webcam*.mp4',cv.VideoWriter_fourcc(*'mp4v'),self.fps,(640,480))
        webcam_writer = cv.VideoWriter('outputs/webcam/webcam*.mp4',cv.VideoWriter_fourcc(*'mp4v'),30,(640,480))

        timestamp = []
        pressure = []
        polaris = []
        frankajnt = []

        # for counter in range(self.tot_frames): 
        for counter in range(30*2*60):
            ret, frame = webcam.read()
            if not ret: break 
            webcam_writer.write(frame)
            cv.imshow('Webcam Binary',frame)

            timestamp.append(time.time())
            pressure_val = np.array(self.pressure_snsr_val,self.pump_state)
            pressure.append(pressure_val)
            polaris.append(self.polaris_pos)
            frankajnt.append(self.franka_pos)

            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break    
        webcam.release()                                    
        webcam_writer.release()
        cv.destroyAllWindows()

        # save timestamps as csv 
        header = ['Counter','Timestamp','Pressure (kPa)','Polaris Tx','Polaris Ty','Polaris Tz','Polaris Error','Polaris Q0','Polaris Qx','Polaris Qy','Polaris Qz','Franka EE','','','','','','','']
        np.savetxt('outputs/webcam/timestamp*.csv', np.array(counter, timestamp, pressure, polaris, frankajnt), header=header, delimiter=',')
        
    def fibrescope_execute(self):
        print("Fibrescope execution selected")
        
        fibrescope = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        print("Using device ", fibrescope.GetDeviceInfo().GetModelName())
        fibrescope.Open() # is this needed?? 
        fibrescope.Width.SetValue(640)
        fibrescope.Height.SetValue(480)
        # also need to set other setting values... Gain and exposure. 
        # !!!
        # others keep default
        # fibrescope.Open() # Open the camera 
        # fibrescope.AcquisitionMode.SetValue('SingleFrame')
        # fibrescope.StartGrabbing(pylon.GrabStrategy_OneByOne)
        # self.ref_frame = fibrescope.RetrieveResult()
        # fibrescope.StopGrabbing()
        # time.sleep(2)
        fibrescope.AcquisitionFrameRateAbs.SetValue(self.fps)
        fibre_writer = cv.VideoWriter('fibrescope2/videos/fibrescope*.mp4',cv.VideoWriter_fourcc(*'mp4v'),self.fps,(fibrescope.Width.GetValue(),fibrescope.Height.GetValue()))
        fibrescope.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        
        timestamp = []
        pressure = []
        polaris = []
        frankajnt = []

        for counter in range(self.tot_frames):
            # if not ret: break
            # do the thing
            frame = fibrescope.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            # if not ret: break
            if frame.GrabSucceeded():
                frame = converter.Convert(frame)
            # frame = automation.filter_frame_fibrescope(frame) # bonarize frame
                img = frame.GetArray()
                fibre_writer.write(img)
                cv.imshow('Recording',img)

                timestamp.append(time.time())
                # pressure_val = np.array(self.pressure_snsr_val,self.pump_state)
                # pressure.append(pressure_val)
                # polaris.append(self.polaris_pos)
                # frankajnt.append(self.franka_pos)

            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break
        fibrescope.StopGrabbing()
        fibre_writer.release()
        cv.destroyAllWindows()
        
        # save timestamps as csv
        # header = ['Counter','Timestamp','Pressure (kPa)','Polaris Tx','Polaris Ty','Polaris Tz','Polaris Error','Polaris Q0','Polaris Qx','Polaris Qy','Polaris Qz','Franka EE','','','','','','','']
        # np.savetxt('outputs/fibrescope/timestamp*.csv', np.array(counter, timestamp, pressure, polaris, frankajnt),header=header,delimiter=',')

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

    