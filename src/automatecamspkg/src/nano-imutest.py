#!/usr/bin/env python3

import socket
import rospy
from std_msgs.msg import Float32
import netifaces as ni

# HOST = "143.167.51.103"  # Standard loopback interface address (localhost)
HOST = ni.ifaddresses('enp0s31f6')[ni.AF_INET][0]['addr'] # get pc host automatically. 
print("Current IP-addr HOST: ", HOST)
PORT = 2055  # Port to listen on (non-privileged ports are > 1023)
FPS = 10 # Hz

roll_x, pitch_y, yaw_z = 0.0, 0.0, 0.0

def gyro_integrate(d_roll, d_pitch, d_yaw): # input is rate of change - i.e. angular velocity
    global roll_x, pitch_y, yaw_z 
    dt = 1.0/FPS
    
    # integration of values 
    roll_x = roll_x + d_roll * dt 
    pitch_y = pitch_y + d_pitch * dt
    yaw_z = yaw_z + d_yaw * dt 
    
    return roll_x, pitch_y, yaw_z # output is angle

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    rospy.init_node('imutest', anonymous=True) # ros node init
    rotX_pub = rospy.Publisher('rotX', Float32, queue_size=10)
    rotY_pub = rospy.Publisher('rotY', Float32, queue_size=10)
    rate = rospy.Rate(FPS) # 10 Hz
    
    check = 0
    # s.TCPServer.allow_reuse_address = True
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    s.bind((HOST, 2055))
    s.listen()
    try:
        # print('listening...')
        conn, addr = s.accept()
    except Exception as e:
        print(e)
    with conn:
        print(f"Connected by {addr}")
        while (not rospy.is_shutdown()):
            data = conn.recv(1024).decode('utf-8')

            try:
                check = check+1
                # print(data)
                values = data.split(',')
                X_raw = float(values[0])
                Y_raw = float(values[1])
                
                rotX, rotY, rotZ = gyro_integrate(X_raw, Y_raw, 0)
                
                if rotX == 404.0404 or rotY == 404.0404:
                    print("ERROR: No data received")
                    pass
                
                if check == 1: print('Publishing now...') # publishing check
                
                rotX_pub.publish(rotX)
                rotY_pub.publish(rotY)
                # rate.sleep()
                # print('Received x rotation value:', rotX)
                # print('Received y rotation value:', rotY)
                # print("\n")
            
            except ValueError as e: # this allows the code to execute with the data being passed as a string, and converts to float later.
                # print(e)
                pass
            except rospy.ROSInterruptException:
                break

    conn.close()
exit()
