# franka-datacollect-ws-ros-mcp
Data collecting from MCP using the franka arm. Data collection from webcam and fibrescope. 

# Steps to run the automation file `automatecams.py`
1. Ensure all devices are connected via the network switch.
2. Ensure the IP addresses are all set properly under ipv4 settings, refer to notion for this. 
3. Franka set-up:
   1. Go to the browser and franka's ip address: https://173.16.0.2/desk/ 
   2. To control via code: click `Activate FCI`
4. In current terminal (1):
   1. Activate virtual env: `source mcp_datacollect_env/bin/activate`
   2. Source ros1 `source /opt/ros/noetic/setup.bash`
   3. Source the workspace `source devel/setup.bash`
5. Terminal 2 - run ros core: 
   1. Source the ros environment: `source /opt/ros/noetic/setup.bash`
   2. Run ros core: `roscore`
6. Terminal 3 - Run rosserial for arduino publishing sensor values: 
   1. Source the environment: `source /opt/ros/noetic/setup.bash`
   2. Then: `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600`
   3. To see published values, open a terminal for each topic and source it, then: (ALTERNATIVELY: USE `PLOTJUGGLER`)
      1. run the topic: `rostopic echo /chatter # change topic name accordingly`
7. Terminal 4 - running the ros bridge: 
   1. cd into the bridge workspace: `cd ~/ros1_bridge_ws/`
   2. Source ros1 environment first: `source /opt/ros/noetic/setup.bash`
   3. Then source ros2 environment: `source /opt/ros/foxy/setup.bash`
   4. Source the workspace: `source install/setup.bash`
   5. Run the bridge: `ros2 run ros1_bridge dynamic_bridge`
8. Terminal 5 - polaris pubisher: 
   1. run polaris publisher package in this workspace `rosrun ndisensor stray_position_pub 169.254.158.253`
9. Terminal 6 - franka publisher: 
   1.  cd into franka ros2 workspace: `cd ros2-franka-ws`
   2.  Source ros2: `source /opt/ros/noetic/setup.bash`
   3.  Source the workspace: `source install/setup.bash`  
   4.  To launch the motion file use: `ros2launch motionmannequin motionmannequin.launch.py robot_ip:=173.16.0.2`
   5.  This opens RVIZ. Next, follow steps below: 
       1.  
10. Back to terminal 1: 
    1.  Since `automatecams.py` is a ROS package in the ws, use `rosrun automatecamspkg` to run the package. 


# Issues
1. polaris publishing callback error - needs fixing
2. fibrescope sometimes does not work. to solve this open camera with pylon and close > run code again. 

