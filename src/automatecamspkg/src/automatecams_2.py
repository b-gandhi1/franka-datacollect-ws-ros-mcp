#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Int32 # , Float64MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2 as cv # pip install opencv-python
from pypylon import pylon # cd pypylon > pip install .
from pypylon import genicam
import time
import os
import pandas as pd # pip install pandas
import sys
from movella_dot.msg import DotSensorMsg

# import roboticstoolbox as rtb # pip install roboticstoolbox-python
# from pyquaternion import Quaternion # pip install pyquaternion

print('imports done successfully!')

# init global variables
pressure_snsr_val = 0.0
pump_state = 0.0
franka_position = np.empty(7)
cam_trig = 0
rotX = 0.0
rotY = 0.0
rotZ = 0.0

# define constant parameters - in CAPS
FPS = 10.0 # 10 fps
TOT_FRAMES = int(FPS*60) # 10 fps, 60 secs (1 min) long recording
# TOT_FRAMES = int(FPS*5) # 5 secs
DESIREDWIDTH = 640
DESIREDHEIGHT = 480

class automation():
    # IMU NANO 33 IOT SUB -------
    # def imuX_callback(data):
    #     global rotX
    #     rotX = data.data
    # def imuY_callback(data):
    #     global rotY
    #     rotY = data.data
    # def imuZ_callback(data):
    #     global rotZ
    #     rotZ = data.data
    # def imu_sub():
    #     try:
    #         rospy.Subscriber('rotX', Float32, automation.imuX_callback)
    #         rospy.Subscriber('rotY', Float32, automation.imuY_callback)
    #         rospy.Subscriber('rotZ', Float32, automation.imuZ_callback)
    #     except rospy.ROSInterruptException:
    #         exit()
    # -------------------
    # IMU XSENS SUB -----
    def xsens_callback(data):
        global rotX, rotY, rotZ
        rotX = data.Dot_Sens3.x
        rotY = data.Dot_Sens3.y
        rotZ = data.Dot_Sens3.z
    def xsens_sub():
        try:
            rospy.Subscriber('DOT_Pos', DotSensorMsg, automation.xsens_callback)
        except rospy.ROSInterruptException:
            exit()
    # ARDUINO UNO SUB -------
    def arduino_pressure_callback(data):
        global pressure_snsr_val
        pressure_snsr_val = data.data
    def arduino_pumpstatus_callback(data):
        global pump_state
        pump_state = data.data
    def arduino_sub():
        try:
            rospy.Subscriber('pressure_val', Float32, automation.arduino_pressure_callback)
            rospy.Subscriber('pump_state', Float32, automation.arduino_pumpstatus_callback)
        except rospy.ROSInterruptException:
            exit()
    # --------------------
    # FRANKA POS SUB -----
    def franka_pos_callback(data):
        global franka_position
        posTx = data.pose.position.x
        posTy = data.pose.position.y
        posTz = data.pose.position.z
        posRx = data.pose.orientation.x
        posRy = data.pose.orientation.y
        posRz = data.pose.orientation.z
        posRw = data.pose.orientation.w
        # above vars are actually joint angles for the 7 joints on the panda arm
        # joints = np.array([posTx,posTy,posTz,posRx,posRy,posRz,posRw])
        # joints = np.array([0,0,0,0,0,0,0])
        # use forward kinematics to extract end-effector position: 
        # panda = rtb.models.DH.Panda() # define robot arm DH convention craigs method
        # T_base_ee = np.asarray(panda.fkine(joints)) # calculate forward kinematics, and convert from tuple to array. 
        # T_base_mannequin = ...
        # convert transformation matrix to quaternion format
        # franka_pos_quat= Quaternion(matrix=T_base_ee)
        # franka_position = np.append(franka_pos_quat.real,franka_pos_quat.imaginary)
        franka_position = np.array([posTx,posTy,posTz,posRx,posRy,posRz,posRw])
        # print(franka_position)
    def franka_pos_sub():
        try: 
            rospy.Subscriber('franka_ee_pos', PoseStamped, automation.franka_pos_callback)
        except rospy.ROSInterruptException:
            exit()
    # --------------------
    # FRANKA TRIG SUB ----
    def franka_trigger_callback(data):
        global cam_trig
        cam_trig = data.data
    def franka_trigger_sub():
        try:
            rospy.init_node('franka_trigger_camera', anonymous=True)
            rospy.Subscriber('camera_trigger', Int32, automation.franka_trigger_callback) 
        except rospy.ROSInterruptException:
            exit()
    # ---------------------
    # CAMERAS -------------
    def webcam_execute():
        print("Webcam execution selected.")
        
        # storage vars
        counter_save = []
        timestamp = []
        pressure_snsr_vals = []
        pump_state_vals = []
        frankapos_vals = np.empty(7)
        
        webcam = cv.VideoCapture(4) # usb logitech webcam. HPC = 0, laptop = 4/0
        
        if not webcam.isOpened(): 
            print("ERROR: Unable to open webcam.")
            exit()
        
        webcam.set(cv.CAP_PROP_FRAME_WIDTH, DESIREDWIDTH) # 640
        webcam.set(cv.CAP_PROP_FRAME_HEIGHT, DESIREDHEIGHT) # 480
        webcam.set(cv.CAP_PROP_FPS, FPS) # 10.0
        
        web_root = os.path.join('src/automatecamspkg/src/outputs/webcam')
        web_filename = 'webcam-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.mp4'
        web_writer = cv.VideoWriter(os.path.join(web_root,web_filename),cv.VideoWriter_fourcc(*'mp4v'),FPS,(DESIREDWIDTH,DESIREDHEIGHT),True)
        
        curr_time = time.time()
        
        for counter in range(TOT_FRAMES):
            ret, frame = webcam.read()
            if not ret: break
            
            cv.imshow("webcam recording", frame)
            web_writer.write(frame)
                    
            # run subscribers alongside
            automation.arduino_sub()
            automation.franka_pos_sub()
            automation.imu_sub()
            
            counter_save.append(counter)
            timestamp.append(time.strftime("%H-%M-%S"))
            pressure_snsr_vals.append(pressure_snsr_val)
            pump_state_vals.append(pump_state)
            frankapos_vals = np.vstack((frankapos_vals, franka_position)) # append for numpy to stack rows
            
            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break
            
            # time.sleep(1/FPS)
            elapsed_time = time.gmtime(time.time() - curr_time)
            elapsed_time_format = "{:02d}:{:02d}".format(elapsed_time.tm_min,elapsed_time.tm_sec)
            print('Elapsed time: ',elapsed_time_format,' Frame: ',counter,'/',TOT_FRAMES,end="\r")


        web_writer.release()
        webcam.release()
        cv.destroyAllWindows()
        print("Recording has finished. Saving data...")
        
        # conversions
        # frankapos_vals = np.asarray(frankapos_vals) # tuple to array
        frankapos_vals = frankapos_vals[1:,:]
        print('frankavals shape: ',np.shape(frankapos_vals))
        print('counter_save shape: ',np.shape(counter_save))
        # save csv file
        header = ['Counter','Timestamp','Pressure (kPa)','Pump State','Franka Tx','Franka Ty','Franka Tz','Franka Rx','Franka Ry','Franka Rz','Franka Rw']
        save_arr = np.array([counter_save,timestamp,pressure_snsr_vals,pump_state_vals,frankapos_vals[:,0],frankapos_vals[:,1],frankapos_vals[:,2],frankapos_vals[:,3],frankapos_vals[:,4],frankapos_vals[:,5],frankapos_vals[:,6]])
        
        root = os.path.join('~/franka-datacollect-ws-ros-mcp/src/automatecamspkg/src/outputs/webcam')
        filename = 'webcam-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.csv'
        
        df = pd.DataFrame(np.transpose(save_arr), columns=header)
        df.to_csv(os.path.join(root, filename), header=header, index = True, sep=',', mode='a')
        
        print("Finished saving data.")
        
    def fibrescope_execute():
        print("Fibrescope execution selected.")
        
        # storage vals
        counter_save = []
        timestamp = []
        pressure_snsr_vals = []
        pump_state_vals = []
        frankapos_vals = np.empty(7)
        rotXvals, rotYvals, rotZvals = [], [], []
        
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
        
        fib_root = os.path.join('src/automatecamspkg/src/outputs/imu_mcp_fusion')
        fib_filename = 'fibrescope-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.mp4'
        fib_writer = cv.VideoWriter(os.path.join(fib_root,fib_filename),cv.VideoWriter_fourcc(*'mp4v'),FPS,(DESIREDWIDTH,DESIREDHEIGHT),0)
        
        # fibrescope.TLParamsLocked = True # grab unlock
        fibrescope.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        
        # converter = pylon.ImageFormatConverter()
        # converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        # converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        curr_time = time.time()

        for counter in range(TOT_FRAMES):
            frame = fibrescope.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            
            if not frame.GrabSucceeded(): 
                print("ERROR: Unable to grab frame.")
                exit()
            # bgr_img = converter.Convert(frame)
            # bgr_img = bgr_img.GetArray()
            bgr_img = frame.Array

            bgr_img =cv.resize(bgr_img,(DESIREDWIDTH,DESIREDHEIGHT)) # resize, not ideal method but seems like the only way left...
            
            cv.imshow("Fibrescope recording", bgr_img)
            fib_writer.write(bgr_img)
            
            # run subscribers alongside
            automation.arduino_sub()
            automation.franka_pos_sub()
            automation.xsens_sub()
            
            counter_save.append(counter)
            timestamp.append(time.strftime("%H-%M-%S"))
            pressure_snsr_vals.append(pressure_snsr_val)
            pump_state_vals.append(pump_state)
            frankapos_vals = np.vstack((frankapos_vals, franka_position)) # append for numpy to stack rows
            rotXvals.append(rotX)
            rotYvals.append(rotY)
            rotZvals.append(rotZ)
            
            if cv.waitKey(10) & 0xFF == ord('q'):
                print('Quitting...')
                break
            
            # print('Iter: ',counter,'/',TOT_FRAMES)
            elapsed_time = time.gmtime(time.time() - curr_time)
            elapsed_time_format = "{:02d}:{:02d}".format(elapsed_time.tm_min,elapsed_time.tm_sec)
            print('Elapsed time: ',elapsed_time_format,' Frame: ',counter,'/',TOT_FRAMES, end="\r")
            # time.sleep(1/FPS) # this is not needed, it messes with FPS.. 
            
        fib_writer.release()
        fibrescope.StopGrabbing()
        fibrescope.Close()
        cv.destroyAllWindows()
        
        print("Recording has finished. Saving data...")
        
        # conversions
        # frankapos_vals = np.asarray(frankapos_vals) # tuple to array
        frankapos_vals = frankapos_vals[1:,:]
        
        # save csv file
        header = ['Counter','Timestamp','Pressure (kPa)','Pump State','IMU X','IMU Y', 'IMU Z','Franka Tx','Franka Ty','Franka Tz','Franka Rx','Franka Ry','Franka Rz','Franka Rw']
        save_arr = np.array([counter_save,timestamp,pressure_snsr_vals,pump_state_vals,rotXvals,rotYvals,rotZvals,frankapos_vals[:,0],frankapos_vals[:,1],frankapos_vals[:,2],frankapos_vals[:,3],frankapos_vals[:,4],frankapos_vals[:,5],frankapos_vals[:,6]])
        
        root = os.path.join('~/franka-datacollect-ws-ros-mcp/src/automatecamspkg/src/outputs/imu_mcp_fusion')
        filaname = 'fibrescope-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.csv'
        
        df = pd.DataFrame(np.transpose(save_arr), columns=header)
        df.to_csv(os.path.join(root, filaname), header=header, index = True, sep=',', mode='a')
        
        print("Finished saving data.")
        
    def wrong_input():
        print("This is the default case. Input not recognised, please try again.")
        
    # ---------------------
# MAIN --------------------
def main(case_select):

    switcher = {
        'w': automation.webcam_execute,
        'f': automation.fibrescope_execute,
    }
    chosen_case = switcher.get(case_select, automation.wrong_input)
    
    automation.franka_trigger_sub() # camera trigger from franka, sent when franka starts moving
    print('Waiting for cam trigger from franka...')
    while(cam_trig != 1):
        # print(cam_trig)
        pass
    
    try:
        chosen_case()
    except KeyboardInterrupt:
        print('*****ERROR: Manually interrupted*****')
        pass
# ------------------------- 
if __name__ == '__main__':
    # device = 0
    device = sys.argv[1]
    # device = rospy.get_param('auto_selected_cam/cam_select') # node_name/argsname
    # if device == 0:
        # device = input("Enter relevant letter for camera selection: 'w' - webcam; 'f' - fibrescope : ")
    # get parameter sys python
    main(device)
