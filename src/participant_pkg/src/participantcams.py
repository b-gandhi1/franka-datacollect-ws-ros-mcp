#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32 #, Int32 , Float64MultiArray, Float32MultiArray
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

# init input variables 
motion_type = ""
num = None

# define constant parameters - in CAPS
FPS = 10.0 # 10 fps
TOT_FRAMES = int(FPS*60) # 10 fps, 60 secs (1 min) long recording
# TOT_FRAMES = int(FPS*5) # 5 secs
DESIREDWIDTH = 640
DESIREDHEIGHT = 480

class automation():
    
    def __init__(self):
        # init obj variables
        self.pressure_snsr_val = 0.0
        self.pump_state = 0.0
        self.rotX = 0.0
        self.rotY = 0.0
        self.rotZ = 0.0
        self.polaris_pos = np.empty(6)
        
        # subscribers to call callbacks: 
        rospy.Subscriber('Marker_Pos', PoseStamped, self.polaris_callback)
        rospy.Subscriber('DOT_Pos', DotSensorMsg, self.xsens_callback)
        rospy.Subscriber('pressure_val', Float32, self.arduino_pressure_callback)
        rospy.Subscriber('pump_state', Float32, self.arduino_pumpstatus_callback)
        
        self.fibrescope_execute()
        
    # IMU XSENS SUB -----
    def xsens_callback(self,data):
        self.rotX = data.Dot_Sens3.x
        self.rotY = data.Dot_Sens3.y
        self.rotZ = data.Dot_Sens3.z
    # def xsens_sub():
    #     try:
    #         rospy.Subscriber('DOT_Pos', DotSensorMsg, automation.xsens_callback)
    #     except rospy.ROSInterruptException:
    #         exit()
    
    # --------------------
    
    # ARDUINO UNO SUB -------
    def arduino_pressure_callback(self,data):
        self.pressure_snsr_val = data.data
    def arduino_pumpstatus_callback(self,data):
        self.pump_state = data.data
    # def arduino_sub():
    #     try:
    #         rospy.Subscriber('pressure_val', Float32, automation.arduino_pressure_callback)
    #         rospy.Subscriber('pump_state', Float32, automation.arduino_pumpstatus_callback)
    #     except rospy.ROSInterruptException:
    #         exit()
    # --------------------
    
    # POLARIS SUB ----
    def polaris_callback(self,data):
        Tx = data.pose.position.x
        Ty = data.pose.position.y
        Tz = data.pose.position.z
        Rx = data.pose.orientation.x # roll x
        Ry = data.pose.orientation.y # pitch y
        Rz = data.pose.orientation.z # yaw z 
        # rotations in euler format already 
        self.polaris_pos = np.array([Tx,Ty,Tz,Rx,Ry,Rz]) 
        
    # def polaris_sub():
    #     try:
    #         rospy.Subscriber('Marker_Pos', Float32, automation.polaris_callback)
    #     except rospy.ROSInterruptException:
    #         exit()
    # --------------------
    
    # FIBRESCOPE -------------
    def fibrescope_execute(self):
        print("Fibrescope execution selected.")
        
        # storage vals
        counter_save = []
        timestamp = []
        pressure_snsr_vals = []
        pump_state_vals = []
        polaris_vals = np.empty(6)
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
        
        fib_root = os.path.join('src/participant_pkg/src/outputs/participant'+str(num))
        fib_filename = 'fibrescope-'+motion_type+'-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.mp4'
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
            
            counter_save.append(counter)
            timestamp.append(time.strftime("%H-%M-%S"))
            pressure_snsr_vals.append(self.pressure_snsr_val)
            pump_state_vals.append(self.pump_state)
            polaris_vals = np.vstack((polaris_vals, self.polaris_pos)) # append for numpy to stack rows
            rotXvals.append(self.rotX)
            rotYvals.append(self.rotY)
            rotZvals.append(self.rotZ)
            
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
        polaris_vals = polaris_vals[1:,:]
        
        # save csv file
        header = ['Counter','Timestamp','Pressure (kPa)','Pump State','IMU X','IMU Y', 'IMU Z','Polaris Tx','Polaris Ty','Polaris Tz','Polaris Rx','Polaris Ry','Polaris Rz']
        save_arr = np.array([counter_save,timestamp,pressure_snsr_vals,pump_state_vals,rotXvals,rotYvals,rotZvals,polaris_vals[:,0],polaris_vals[:,1],polaris_vals[:,2],polaris_vals[:,3],polaris_vals[:,4],polaris_vals[:,5]])
        
        root = os.path.join('~/franka-datacollect-ws-ros-mcp/src/participant_pkg/src/outputs/participant'+num)
        filaname = 'fibrescope-'+motion_type+'-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.csv'
        
        df = pd.DataFrame(np.transpose(save_arr), columns=header)
        df.to_csv(os.path.join(root, filaname), header=header, index = True, sep=',', mode='a')
        
        print("Finished saving data.")        
    # ---------------------
# MAIN --------------------
def main():
    
    try:
        rospy.init_node('participant_subs', anonymous=True)
        automation() # or obj=automation()automation
                
    except KeyboardInterrupt:
        print('*****ERROR: Manually interrupted*****')
        exit()
# ------------------------- 
if __name__ == '__main__':
    num = sys.argv[1] # participant number
    motion_type = sys.argv[2] # pitch | roll | trans
    # device = rospy.get_param('auto_selected_cam/cam_select') # node_name/argsname
    # if device == 0:
        # device = input("Enter relevant letter for camera selection: 'w' - webcam; 'f' - fibrescope : ")
    # get parameter sys python
    main()
