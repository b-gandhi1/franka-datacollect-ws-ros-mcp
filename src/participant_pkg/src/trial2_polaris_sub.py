#!/usr/bin/env python3

import rospy
# from std_msgs.msg import Float32 #, Int32, Float64MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import sys
import os
import pandas as pd
import time

FPS = 10.0 # 10 fps
TOT_FRAMES = int(FPS*60) # 10 fps, 60 secs (1 min) long recording

motion_type = ""
num = None

class automation_trial():    
    def __init__(self):
        self.polaris_pos = np.empty(6)
        rospy.Subscriber('Marker_Pos', PoseStamped, self.polaris_callback)
        print("hi")
        self.recording_func_trial()
        
    def polaris_callback(self,data):
        Tx = data.pose.position.x
        Ty = data.pose.position.y
        Tz = data.pose.position.z
        Rx = data.pose.orientation.x # roll x
        Ry = data.pose.orientation.y # pitch y
        Rz = data.pose.orientation.z # yaw z 
        # rotations in euler format already 
        self.polaris_pos = np.array([Tx,Ty,Tz,Rx,Ry,Rz])         
        
    def recording_func_trial(self):
        
        print("Fibrescope execution selected.")
        
        # for counter in range(TOT_FRAMES):
        while not rospy.is_shutdown():         
            # if KeyboardInterrupt:
            #     print('KeyboardInterrupt. Quitting...')
            #     break
            
            # print('Iter: ',counter,'/',TOT_FRAMES)
            # elapsed_time = time.gmtime(time.time() - curr_time)
            # elapsed_time_format = "{:02d}:{:02d}".format(elapsed_time.tm_min,elapsed_time.tm_sec)
            # print('Elapsed time: ',elapsed_time_format,' Frame: ',counter,'/',TOT_FRAMES, end="\r")
            time.sleep(1/FPS) # this is not needed, it messes with FPS.. 
            print("Polaris position: ", self.polaris_pos[3:6], end="\r")
        # print("Recording has finished. Saving data...")
        
        # # frankapos_vals = np.asarray(frankapos_vals) # tuple to array
        # polaris_vals = polaris_vals[1:,:]
        
        # # save csv file
        # header = ['Polaris Tx','Polaris Ty','Polaris Tz','Polaris Rx','Polaris Ry','Polaris Rz']
        # save_arr = np.array([polaris_vals[:,0],polaris_vals[:,1],polaris_vals[:,2],polaris_vals[:,3],polaris_vals[:,4],polaris_vals[:,5]])
        
        # root = os.path.join('~/franka-datacollect-ws-ros-mcp/src/participant_pkg/src/outputs/participant'+num)
        # filaname = 'fibrescope-'+motion_type+'-'+time.strftime("%d-%b-%Y--%H-%M-%S")+'.csv'
        
        # df = pd.DataFrame(np.transpose(save_arr), columns=header)
        # df.to_csv(os.path.join(root, filaname), header=header, index = True, sep=',', mode='a')
        
        # print("Finished saving data.")
def main():
    try:
        rospy.init_node("Polaris_subscriber", anonymous=True)
        obj = automation_trial()
        # rospy.spin()
    except KeyboardInterrupt:
        print('\n*****ERROR: Manually interrupted*****')
        exit()
    
if __name__ == '__main__':
    # num = sys.argv[1] # participant number
    # motion_type = sys.argv[2] # pitch | roll | trans
    # device = rospy.get_param('auto_selected_cam/cam_select') # node_name/argsname
    # if device == 0:
        # device = input("Enter relevant letter for camera selection: 'w' - webcam; 'f' - fibrescope : ")
    # get parameter sys python
    main()