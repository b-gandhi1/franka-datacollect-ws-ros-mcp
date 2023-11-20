# franka-datacollect-ws-ros-mcp
Data collecting from MCP using the franka arm. Data collection from webcam and fibrescope. 

## Steps to run the automation file `automatecams.py`
1. Ensure all devices are connected via the network switch.
2. Ensure the IP addresses are all set properly under ipv4 settings, refer to notion for this. 
3. Franka set-up:
   1. Go to the browser and franka's ip address: https://173.16.0.2/desk/ 
   2. To control via code: click `Activate FCI`
4. In current terminal (1):
   1. `cd` into the `franka-datacollect-ws-ros-mcp`
   2. Activate virtual env: `source mcp_datacollect_env/bin/activate`
   3. Source ros1 `source /opt/ros/noetic/setup.bash`
   4. Source the workspace `source devel/setup.bash`
   5. When set-up ready, execute file:
      - Launch file: Since automatecams.py is a ROS package in the ws, use roslaunch `roslaunch launch_pkg automation.launch cam_select:=<s>` to run the package, where 's' = 'w' or 'f' for webcam or fibrescope respectively. 
        - THERE IS AN ISSUE WITH THIS. IT DOES NOT SAVE VIDEOS. 
      - **Another way**: `rosrun automatecamspkg automatecams_2.py <s>` where 's' is the same as earlier. 
      -  **Note:** this needs to be executed **before** franka is executed
5. Terminal 2 - run ros core: 
   1. Source the ros environment: `source /opt/ros/noetic/setup.bash`
   2. Run ros core: `roscore`
6. Terminal 3 - Run rosserial for arduino publishing sensor values: 
   1. Source the environment: `source /opt/ros/noetic/setup.bash`
   2. Then: `rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600`
   3. To see published values, open a terminal for each topic and source it, then: (ALTERNATIVELY: USE PLOTJUGGLER `plotjuggler -l mcp_data_collect_plotjuggler_view.xml`)
      1. cd into `franka-datacollect-ws-ros-mcp`. 
      2. Source ros1 source /opt/ros/noetic/setup.bash
      3. Source the workspace source devel/setup.bash
      4. run the topic: `rostopic echo /chatter # change topic name accordingly. chatter=pump_state or pressure_val`
7. Terminal 4 - running the ros bridge: 
   1. cd into the bridge workspace: `cd ~/ros1_bridge_ws/`
   2. Source ros1 environment first: `source /opt/ros/noetic/setup.bash`
   3. Then source ros2 environment: `source /opt/ros/foxy/setup.bash`
   4. Source the workspace: `source install/setup.bash`
   5. Run the bridge: `ros2 run ros1_bridge dynamic_bridge`
8. !! IGNORE !! --- Terminal 5 - polaris pubisher: 
   1. run polaris publisher package in this workspace `rosrun ndisensor stray_position_pub 169.254.158.253`
9.  Terminal 6 - franka publisher: 
    1.  cd into franka ros2 workspace: `cd ~/ros2-franka-ws`
    2.  Source ros2: `source /opt/ros/foxy/setup.bash`
    3.  Source the workspace: `source install/setup.bash`  
    4.  Before executing the code, ensure the gripper and mannequin are in position, and the grip is secure. Use commands for securing grip:
        1.  Open new terminal for gripper: 
            1.  source ros2 and ros2 ws
            2.  Then: `ros2 launch franka_gripper gripper.launch.py robot_ip:=173.16.0.2`
        2.  Then control Gripper: 
            1.  **GRIP OPEN:** `ros2 action send_goal /panda_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.024, max_effort: -1}}"`
            2.  **GRIP SECURE:** `ros2 action send_goal /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.0, force: 40.0, speed: 0.1,  epsilon:{inner: 0.08, outer: 0.08}}"` 
10. To launch the motion file use: `ros2 launch motionmannequin motionmannequin.launch.py robot_ip:=173.16.0.2`
    - **Simulation version:** (fake hardware) use command: `ros2 launch motionmannequin motionmannequin.launch.py robot_ip:=dont_care use_fake_hardware:=true`
      - This opens RVIZ, and robot starts to move. Upon connection, it should send a trigger to the camera automation to start recording. So both should start at the same time. 

## Issues
1. fibrescope sometimes does not work. to solve this open camera with pylon and close > run code again. 
2. sometimes the camera trigger does not get registered, along with other ros2 to ros1 communication. Restart `ros1-bridge` to solve this. 

## Notes:
* To update franka motion, do so manually: 
    1. Set the franka arm in desired position manually. 
    2. Open a terminal in ros2: 
       1. run command: `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=173.16.0.2`
       2. In new tab, run `ros2 topic echo /joint_states`
       3. Get first 7 positions and update the cpp file. 
* Sometimes crashes while running on laptop. Easy fix: connect laptop charger! :) 