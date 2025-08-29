# franka-datacollect-ws-ros-mcp
Data collecting from MCP 

## Steps to run the automation file `automatecams.py`

Data colleciton using the franka arm. Data collection from webcam and fibrescope. 
1. Ensure all devices are connected via the network switch.
2. Ensure the IP addresses are all set properly under ipv4 settings, refer to notion for this. 
3. Franka set-up:
   1. Go to the browser and franka's ip address: https://173.16.0.2/desk/ 
   2. To control via code: click `Activate FCI`
## Terminal for franka automation and data collection
```
cd franka-datacollect-ws-ros-mcp

source ~/Movella_DOT_ROS/devel/setup.bash # THIS IS NEEDED for xsens imu subscriber!!

# check ROS path to ensure this has been included: 
echo $ROS_PACKAGE_PATH 

# NOTE: might need to rebuild workspace adter this step. 

source mcp_datacollect_env/bin/activate # virtual env activate
source /opt/ros/noetic/setup.bash # source ros1
source devel/setup.bash # source workspace

# run command below to execute programme: 
rosrun automatecamspkg automatecams_2.py <s>
# where 's' = 'w' or 'f' for webcam or fibrescope respectively
``` 

Further troubleshooting for sourcing movella_dot into the franka workspace refer to this link: 
https://www.theconstruct.ai/overlaying-ros-workspaces/ 


## Run roscore and the programs depending on this
This includes the following: 
* Rosserial for arduino uno
  * pump status
  * air pressure sensor value (kPa)
* XSENS imu sensor orientation values in x, y, z
* rosbridge - to pass values between ros1 and ros2

```
source /opt/ros/noetic/setup.bash
roscore
```
Next terminal, source ros1 again and then: 
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600


```
New terminal: Movella IMU sensor XSENS
```
cd Movella_DOT_ROS
. devel/setup.bash
rosrun movella_dot ros_movella 
# follow steps as on screen
# this needs roscore running in the background
```
* To see published values, open a terminal for each topic and source it, then: (ALTERNATIVELY: USE PLOTJUGGLER `plotjuggler -l mcp_data_collect_plotjuggler_view.xml`)
    1. cd into `franka-datacollect-ws-ros-mcp`. 
    2. Source ros1 source /opt/ros/noetic/setup.bash
    3. Source the workspace source devel/setup.bash
    4. run the topic: `rostopic echo /chatter # change topic name accordingly. chatter=pump_state or pressure_val`

ROSBRIDGE new terminal: 
```
cd ~/ros1_bridge_ws/
source /opt/ros/noetic/setup.bash # ros1 first
source /opt/ros/foxy/setup.bash # then ros2 
source install/setup.bash # then workspace

ros2 run ros1_bridge dynamic_bridge # final command


```

## franka publisher and motion execute
Sourcing for each terminal
```
cd ~/ros2-franka-ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

### Gripper commands: 
```
ros2 launch franka_gripper gripper.launch.py robot_ip:=173.16.0.2 # launch gripper

ros2 action send_goal /panda_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.022, max_effort: -1}}" # OPEN GRIPPER

ros2 action send_goal /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.0, force: 40.0, speed: 0.1,  epsilon:{inner: 0.08, outer: 0.08}}" # SECURE GRIPPER
```
*Gripper must be secured to mannequin prior to motion execution.* 


### Motion execution
```
# HARDWARE
ros2 run motionmannequin external_force # needs running once to set joint torques
# above line enables issues around payload for moving heavy mannequin head!

# and finally run the motion file:
ros2 launch motionmannequin motionmannequin.launch.py robot_ip:=173.16.0.2

# OR - use shell script for above: 
sh run_prog_commands.sh

# SIMULATION
ros2 launch motionmannequin motionmannequin.launch.py robot_ip:=dont_care use_fake_hardware:=true
```

*Simulation opens RVIZ, and robot starts to move. Upon connection, it should send a trigger to the camera automation to start recording. So both should start at the same time.* 

## Troubleshooting for automatecams
1. fibrescope sometimes does not work. to solve this open camera with pylon and close > run code again. 
2. sometimes the camera trigger does not get registered, along with other ros2 to ros1 communication. Restart `ros1-bridge` to solve this. 
3. RT Kernel: Sometimes ROS crashes while running on laptop. Easy fix: connect laptop charger! :) 
4. If `acm1` selected for arduino board, may create issues. `acm0` preffered. 
   ```
   sudo chmod 666 /dev/ttyACM0 # run in bash
   # then log out and in again into laptop. 
   # try uploading again. 
   ``` 
5. If NOT connecting to server, may be because of IP address changes. It updates every week on laptop. This needs to reflect in code for server and in the python sketch. 

# PARTICIPANT STUDY
Polaris pubisher and subscriber
 
```
rosrun polaris_pkg object_tracking 192.168.1.10 --tools=<path of rom file> # run polaris publisher package

rosrun participant_pkg participantcams.py <participant_num> <motion_type> # run subscribers, motion: pitch | roll | trans

# to test just the polaris subscriber: 
rosrun participant_pkg trial2_polaris_sub # subscriber for only polaris

```

# LIVE DEMO
Run live demo using the following: 
* Firstly, run `roscore`
* Publish arduino vals:  `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600` 
* Finally, run the cameras and motion estimation algorithms: 
  ```

  # For fibrescope, lukas kanade: 
  rosrun live_demo_pkg live_demo_fib_lk.py

  # For webcam 
  rosrun live_demo_pkg live_demo_web_lk.py

  ```
  
* To open plotjuggler for ros1: `rosrun plotjuggler plotjuggler`
  * load the xml file in this directory called: `mcp_livedemo_fib_lk_ros1.xml`
  * or use below command: 
  ```
  rosrun plotjuggler plotjuggler -l mcp_livedemo_fib_lk_ros1.xml
  ```
  * xml file also available for webcam. 
* rqt_plot is another option: `rosrun rqt_plot rqt_plot` but stick with plotjuggler for now. 

# Arduino nano 33 IOT - IMU tests !IGNORE!
*This is no longer being used, XSENS IMU sensor is being used instead now.*
* Set-up steps: 
  1. Connect PC via ethernet
  2. Enable hotspot
  3. Ensure the credentials match thr ones in the arduino code
  4. Ensure the firewall allows the arduino device through. Arduino ip addr: `10.42.0.88`. 
  5. Connect Arduino via USB to power it. 
* Linux terminal: 
```
cd ~/franka-datacollect-ws-ros-mcp
. /opt/ros/noetic/setup.bash # source ros
. devel/setup.bash
rosrun automatepkg imutest.py # Run the server (which is also the publisher for ROS)
# start running the automatecams_2.py once this starts publishing. 

```

# Notes 
## video output
* in grayscale but read as rgb then convert to grayscale
* flips required as needed. 