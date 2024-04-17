#!/usr/bin/env python3

import socket
import rospy
from std_msgs.msg import Float32

HOST = "143.167.51.103"  # Standard loopback interface address (localhost)
PORT = 2055  # Port to listen on (non-privileged ports are > 1023)
FPS = 10 # Hz

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    rospy.init_node('imutest', anonymous=True) # ros node init
    rotX_pub = rospy.Publisher('rotX', Float32, queue_size=10)
    rotY_pub = rospy.Publisher('rotY', Float32, queue_size=10)
    rate = rospy.Rate(FPS) # 10 Hz
    
    # s.TCPServer.allow_reuse_address = True
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    s.bind(("143.167.51.103", 2055))
    s.listen()
    try:
        # print('listening...')
        conn, addr = s.accept()
    except Exception as e:
        print(e)
    with conn:
        print(f"Connected by {addr}")
        while (not rospy.is_shutdown()):
            data = conn.recv(1024).decode('utf-8').strip()

            try:
                values = data.split(',')
                rotX = float(values[0])
                rotY = float(values[1])
                
                if rotX == 404.0404 or rotY == 404.0404:
                    print("ERROR: No data received")
                    pass

                rotX_pub.publish(rotX)
                rotY_pub.publish(rotY)
                rate.sleep()
                # print('Received x rotation value:', rotX)
                # print('Received y rotation value:', rotY)
                # print("\n")
            
            except ValueError: # this allows the code to execute with the data being passed as a string, and converts to float later.
                pass
            except rospy.ROSInterruptException:
                break

    conn.close()
exit()
