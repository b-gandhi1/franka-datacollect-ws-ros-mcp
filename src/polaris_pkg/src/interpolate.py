import rospy
import numpy
from cam_motion.msg import Force_msg
from scipy.interpolate import interp1d

def original_callback(data):
    global last_time, last_value
    current_time = rospy.get_time()  # Get the time when the message was received
    current_value = data.normal_force.data  # Get the original data value
    
    # Interpolate data using linear interpolation
    interpolation_function = interp1d([last_time, current_time], [last_value, current_value], kind='linear')
    interpolated_times = [last_time + i * 0.1 * (current_time - last_time) for i in range(1, 8)]
    interpolated_values = [interpolation_function(time) for time in interpolated_times]

    for value in interpolated_values:
        msg = Force_msg()
        msg.normal_force.data = value
        msg.header.stamp = rospy.Time.now()
        upsampled_data_pub.publish(msg)
        
    # Update last_time and last_value for next interpolation
    last_time = current_time
    last_value = current_value

def main():
    
    rospy.init_node('upsampling_node')
    
    global last_time, last_value
    last_time = rospy.get_time()  # Initialize last_time
    last_value = 0.0  # Initialize last_value
    
    # Subscribe to the original topic
    original_topic = "/gripper_distance"
    rospy.Subscriber(original_topic, Force_msg, original_callback)
    
    # Publish the upsampled data on a new topic
    upsampled_topic = "/upsampled_topic"
    global upsampled_data_pub
    upsampled_data_pub = rospy.Publisher(upsampled_topic, Force_msg, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()