#!/usr/bin/env python3
# above line is compulsary for ROS environments only. 
# -*- coding: utf-8 -*-
# import sys
# sys.path.append('/usr/local/lib/python3.7/site-packages')
# import multiprocessing as mp
import rospy
from std_msgs.msg import Float32, Int32 # , Float64MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2 as cv
from pypylon import pylon
from pypylon import genicam
import time
import os
# import sys
import pandas as pd

print('imports done successfully!')

pressure_snsr_val = 0.0
pump_state = 0.0
# polaris_pos = 0.0
franka_position = np.zeros(7)
cam_trig = 0

# define constant parameters - in CAPS
# TOT_FRAMES = 30*2*60 # 30 fps, 2 mins.
TOT_FRAMES = 30*5 # 5 secs
FPS = 30.0 # 30 fps
DESIREDWIDTH = 640
DESIREDHEIGHT = 480

class automation():
    def arduino_pressure_callback(data): # pressure value check
        # rospy.loginfo(rospy.get_caller_id() + "Pressure value %s", data.data)
        global pressure_snsr_val
        pressure_snsr_val = data.data

    def arduino_pumpstatus_callback(data): # pump status check
        # rospy.loginfo(rospy.get_caller_id() + "Pump status %s", data.data)
        global pump_state
        pump_state = data.data

    def arduino_sub(): # subscriber for arduino pressure sensor
        try:
            # rospy.init_node("arduino_vals", anonymous=True)
            rospy.Subscriber("pressure_val", Float32, automation.arduino_pressure_callback)
            rospy.Subscriber("pump_state", Float32, automation.arduino_pumpstatus_callback)
            # spin() simply keeps python from exiting until this node is stopped
            # rospy.spin()
        except rospy.ROSInterruptException:
            exit()

    def webcam_execute():
        print("Webcam execution selected.")

        # storage variables
        counter_save = []
        timestamp = []
        pressure = []
        # polaris = []
        frankajnt = []
        
        webcam = cv.VideoCapture(4) # usb logitech webcam
        # webcam = cv.VideoCapture(0) # use internal webcam
        if not (webcam.isOpened()):
            print("Could not open video device")
        webcam.set(cv.CAP_PROP_FRAME_WIDTH, DESIREDWIDTH) # 640
        webcam.set(cv.CAP_PROP_FRAME_HEIGHT, DESIREDHEIGHT) # 480
        webcam.set(cv.CAP_PROP_FPS, FPS)
        
        webcam_root = os.path.join('src/automatecamspkg/src/outputs/webcam')
        webcam_filename = 'webcam-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.mp4'
        webcam_writer = cv.VideoWriter(os.path.join(webcam_root,webcam_filename),cv.VideoWriter_fourcc(*'mp4v'),FPS,(DESIREDWIDTH,DESIREDHEIGHT),True)

        # for counter in range(self.tot_frames): 
        for counter in range(TOT_FRAMES):
            ret, frame = webcam.read()
            if not ret: break 
            
            cv.imshow('Webcam recording',frame)
            webcam_writer.write(frame)
            
            # run subscribers
            automation.arduino_sub()
            # automation.polaris_sub(automation)
            automation.franka_pos_sub()
            
            counter_save.append(counter)
            timestamp.append(time.strftime("%H-%M-%S"))
            pressure_val = np.array([pressure_snsr_val,pump_state])
            pressure.append(pressure_val)
            # polaris.append(polaris_pos)
            frankajnt.append(franka_position)

            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break    
            
        webcam_writer.release()
        webcam.release()  
        cv.destroyAllWindows()

        print('Recoridng finished. Saving data...')
        time.sleep(1)

        # conversions
        pressure = np.asarray(pressure) # tuple to array
        frankajnt = np.asarray(frankajnt)
        # timestamp = np.asarray(timestamp, dtype='datetime64[s]') # time to readable format
        # timestamp = np.asarray(timestamp)

        # save timestamps as npz or csv
        header = ['Counter','Timestamp','Pressure (kPa)','Pump State','Franka EE Tx','Franka EE Ty','Franka EE Tz','Franka EE Rx','Franka EE Ry','Franka EE Rz','Franka EE Rw']
        # np.savez('src/automatecamspkg/src/outputs/webcam/timestamp'+str(datetime.date.today())+'.npz', counter=counter_save, timestamp=timestamp, pressure=pressure, polaris=polaris, frankajnt=frankajnt)
        # fmt = ['%d','%s'] + ['%f'] * 9
        
        filename = os.path.join('timestamp_arr'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.csv')
        save_arr = np.array([counter_save,timestamp,pressure[:,0],pressure[:,1],frankajnt[:,0],frankajnt[:,1],frankajnt[:,2],frankajnt[:,3],frankajnt[:,4],frankajnt[:,5],frankajnt[:,6]])
        
        # with open (filename, 'wt+'):
        #     np.savetxt(os.path.join('~/franka-datacollect-ws-ros-mcp/src/automatecamspkg/src/outputs/webcam',filename), np.transpose(save_arr),header=header,delimiter=',',fmt=fmt)
        
        # use dataframe to save to csv
        df = pd.DataFrame(np.transpose(save_arr), columns=header)
        df.to_csv(os.path.join('~/franka-datacollect-ws-ros-mcp/src/automatecamspkg/src/outputs/webcam',filename), header=header, index=True, sep=',', mode='a') 
        
        # trial save
        # header = 'Counter,Timestamp,Pressure (kPa),Pump state'
        # fmt = ['%d','%s','%f','%f'] # type: [integer, string, float, float]
        # np.savetxt('src/automatecamspkg/src/outputs/webcam/trialsave.csv',np.transpose([counter_save,timestamp,pressure[:,0],pressure[:,1]]),header=header,delimiter=',',fmt=fmt)
        print('data saving done')
        print('counter shape: ', np.shape(counter_save))
        print('timestamp shape: ', np.shape(timestamp))
        print('pressure shape: ', np.shape(pressure))
        print('frankajnt shape: ', np.shape(frankajnt))
        # print(np.hstack((counter_save, timestamp, pressure[:,0], pressure[:,1])))
        
    def fibrescope_execute():
        print("Fibrescope execution selected")
        
        # storage variables 
        counter_save = []
        timestamp = []
        pressure = []
        # polaris = []
        frankajnt = []
        
        # method 1
        # tlFactory = pylon.TlFactory.GetInstance()
        # devices = tlFactory.EnumerateDevices()
        # print(f"devices: {devices}")
        # fibrescope = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())

        #method2
        tlf = pylon.TlFactory.GetInstance()
        # tl = tlf.CreateTl('BaslerGigE')
        # fib_info = tl.CreateDeviceInfo()
        fib_info = pylon.DeviceInfo()
        fib_info.SetIpAddress('169.254.27.123')
        fibrescope = pylon.InstantCamera(tlf.CreateDevice(fib_info))
                
        print("Using device ", fibrescope.GetDeviceInfo().GetModelName())
        fibrescope.Open() 
        
        # set some basic parameters
        fibrescope.Width.SetValue(DESIREDWIDTH) # 640
        fibrescope.Height.SetValue(DESIREDHEIGHT) # 480
        fibrescope.AcquisitionFrameRateAbs.SetValue(FPS)
        fibrescope.GainAuto.SetValue('Off')
        fibrescope.GainRaw.SetValue(200)
        fibrescope.ExposureTimeAbs = 250000.0
        fibrescope.AcquisitionMode.SetValue("Continuous")
        # fibrescope.UserSetSelector = "UserSetMCP"
        # fibrescope.UserSetSave.Execute()
        # fibrescope.UserSetDefaultSelector = "UserSetMCP"
        
        fibre_writer = cv.VideoWriter(os.path.join('src/automatecamspkg/src/automatecamspkg/src/outputs/fibrescope/fibrescope-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.mp4'),cv.VideoWriter_fourcc(*'mp4v'),FPS,(DESIREDWIDTH,DESIREDHEIGHT),True)
        fibrescope.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        
        # converting to opencv bgr format
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed # method 1 for gray2bgr conversion. CONVERSION IS NECESSARY FOR PLAYABLE VIDEO VIA OPENCV!!
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        for counter in range(TOT_FRAMES):
            frame = fibrescope.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if frame.GrabSucceeded():
                bgr_image = converter.Convert(frame)
                bgr_img = bgr_image.GetArray()
                # img = frame.Array # np.array without using covnverter! 
                # bgr_img = cv.cvtColor(img,cv.COLOR_GRAY2BGR) # method 2 for gray2bgr conversion. CONVERSION IS NECESSARY FOR PLAYABLE VIDEO VIA OPENCV!!
                cv.imshow('Fibrescope recording',bgr_img)
                fibre_writer.write(bgr_img) # saving video
                
                # run subscribers
                automation.arduino_sub()
                # automation.polaris_sub(automation)
                automation.franka_pos_sub()
                
                counter_save.append(counter)
                timestamp.append(time.strftime('%H-%M-%S'))
                pressure_val = np.array([pressure_snsr_val,pump_state])
                pressure.append(pressure_val)
                # polaris.append(polaris_pos)
                frankajnt.append(franka_position)

            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break
        fibre_writer.release()
        fibrescope.StopGrabbing()
        fibrescope.Close()
        cv.destroyAllWindows()
        
        print('Recording finished. Saving data...')
        
        time.sleep(1)
        
        # conversions
        pressure = np.asarray(pressure) # tuple to array
        frankajnt = np.asarray(frankajnt)
        # timestamp = np.asarray(timestamp, dtype='datetime64[s]') 
        timestamp = np.asarray(timestamp)
        
        # save timestamps as npz or csv
        header = ['Counter','Timestamp','Pressure (kPa)','Pump State','Franka EE Tx','Franka EE Ty','Franka EE Tz','Franka EE Rx','Franka EE Ry','Franka EE Rz','Franka EE Rw']
        
        save_arr = [counter_save,timestamp,pressure[:,0],pressure[:,1],frankajnt[:,0],frankajnt[:,1],frankajnt[:,2],frankajnt[:,3],frankajnt[:,4],frankajnt[:,5],frankajnt[:,6]]
        filename = os.path.join('timestamp_arr'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.csv')

        # np.savez('src/outputs/fibrescope/timestamp'+str(datetime.date.today())+'.npz', counter=counter_save, timestamp=timestamp, pressure=pressure, polaris=polaris, frankajnt=frankajnt)
        # fmt = ['%d','%s'] + ['%f'] * 9
        # np.savetxt(os.path.join('src/automatecamspkg/src/outputs/fibrescope/timestamp_arr',time.strftime("%d-%b-%Y--%H-%M-%S")+'.csv'), np.transpose(save_arr),header=header,delimiter=',',fmt=fmt)

        # trial save
        # header = ['Counter','Timestamp','Pressure (kPa)']
        # np.savetxt('src/automatecamspkg/src/outputs/fibrescope/trialsave.csv', np.array([counter_save, timestamp, pressure]),header=header,delimiter=',')
        
        # use dataframe to save to csv
        df = pd.DataFrame(np.transpose(save_arr), columns=header)
        df.to_csv(os.path.join('~/franka-datacollect-ws-ros-mcp/src/automatecamspkg/src/outputs/webcam',filename), header=header, index=True, sep=',', mode='a') 
        
        print('data saving done!')
        
    def franka_pos_callback(data):
        # rospy.loginfo(rospy.get_caller_id(), "Franka End Effector Position: %s", data.data)
        global franka_position # ground truth
        posTx = data.pose.position.x
        posTy = data.pose.position.y
        posTz = data.pose.position.z
        posRx = data.pose.orientation.x
        posRy = data.pose.orientation.y
        posRz = data.pose.orientation.z
        posRw = data.pose.orientation.w
        franka_position = np.array([posTx,posTy,posTz,posRx,posRy,posRz,posRw])
        
    def franka_pos_sub(): # subscriber for franka emika robot arm
        # subscribe to topic from ros2, use bridge. 
        try:
            # rospy.init_node('frankaemika', anonymous=True)
            rospy.Subscriber('franka_ee_pos', PoseStamped, automation.franka_pos_callback)
        except rospy.ROSInterruptException:
            exit()

    # def polaris_callback(data):
    #     rospy.loginfo(rospy.get_caller_id(), data.data[0])
    #     global polaris_pos
    #     polaris_pos = data.data[0]

    # def polaris_sub(self): # state of the art comparison, also ground truth backup. 
    #     try: 
    #         rospy.init_node('polaris', anonymous=True)
    #         rospy.Subscriber('Marker_Pos', Float64MultiArray, self.polaris_callback) # topic name = polaris_data - needs to be same in publisher too.
    #         # rospy.spin() 
    #     except rospy.ROSInterruptException:
    #         exit()
        
    def wrong_input():
        print("This is the default case. Input not recognised, please try again.")
            
    def franka_trigger_callback(data):
        # rospy.loginfo(rospy.get_caller_id(), "Camera trigger: %d", data.data)
        global cam_trig # camera trigger
        cam_trig = data.data
        print('cam_trig: ',cam_trig)            

    def franka_trigger_sub():
        try:
            rospy.init_node('franka_trigger_camera', anonymous=True)
            rospy.Subscriber("camera_trigger", Int32, automation.franka_trigger_callback) 
                
        except rospy.ROSInterruptException:
            exit()
        
def main(case_select):
    
    # clsobj = automation()

    switcher = {
        'w': automation.webcam_execute,
        'f': automation.fibrescope_execute,
    }
    # case_select = input("Enter relevant letter for camera selection: 'w' - webcam; 'f' - fibrescope : ")
    # case_number = 1
    # get the function from switcher dictionary
    # if the case number is not found, default to case_default
    chosen_case = switcher.get(case_select, automation.wrong_input)
    
    # print('Waiting for trigger...')
    automation.franka_trigger_sub() # camera trigger from franka, sent when franka starts moving
    print('Waiting for cam trigger from franka...')
    while(cam_trig != 1):
        pass
    
    try:
        chosen_case()
    except KeyboardInterrupt:
        print('*****ERROR: Manually interrupted*****')
        pass
        
    # begin multi process to run chosen_case and polaris together: 
    # arduino_process = mp.Process(target=clsobj.arduino_sub)
    # polaris_process = mp.Process(target=clsobj.polaris_sub)
    # cam_process = mp.Process(target=chosen_case)
    # franka_process = mp.Process(target=clsobj.franka_sub)
    
    # try: 
    #     # start multi process: 
    #     # arduino_process.start()
    #     polaris_process.start()
    #     cam_process.start()
    #     franka_process.start()

    #     # finish multi process:
    #     # arduino_process.join() 
    #     polaris_process.join()
    #     cam_process.join()
    #     franka_process.join()

    #     print('Multi-processes finished.')
    # except KeyboardInterrupt:
    #     print('*****ERROR: Manually interrupted*****')
    #     pass
    
if __name__ == '__main__':
    device = rospy.get_param('/auto_selected_cam/cam_select') # node_name/argsname
    main(device)