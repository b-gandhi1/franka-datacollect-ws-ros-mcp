cd ~/ros2-franka-ws/
source /opt/ros/foxy/setup.bash
source ~/ros2-franka-ws/install/setup.bash
ros2 launch motionmannequin motionmannequin.launch.py robot_ip:=173.16.0.2
