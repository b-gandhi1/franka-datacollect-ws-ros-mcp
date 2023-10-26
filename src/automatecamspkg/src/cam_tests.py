#!/usr/bin/env python

import cv2 as cv
import os
import time
# import rospy
from std_msgs.msg import String

FPS = 20.0 # 30 fps
TOT_FRAMES = int(FPS*5) # 5 secs
DESIREDWIDTH = 640
DESIREDHEIGHT = 480

class test_cam():
    def talker():
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
            # rate.sleep()
    
    def callback(data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    def listener(): 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("chatter", String, test_cam.callback)
    # spin() simply keeps python from exiting until this node is stopped
        # rospy.spin()
    def cam_record():
        cap = cv.VideoCapture(0) # HPC = 0, laptop = 4
        
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
def main():
    test_cam.cam_record()
    
if __name__ == '__main__':
    main()
    print('Done')