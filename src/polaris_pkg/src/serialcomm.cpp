#include <ros/ros.h>
#include <serial/serial.h>
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

#include <string>
#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

// custom message
#include "/home/project/rad-vicon_ws/devel/include/cam_motion/LoadCellMsg.h"
#include "/home/project/rad-vicon_ws/devel/include/cam_motion/Force_msg.h"

struct SensorData {
	float fX;
	float fY;
	float fZ;
	float mX;
	float mY;
	float mZ;
	float temperature;
};

class HexFT {
public:
    ros::NodeHandle nh;
    ros::Publisher read_pub;
    ros::Subscriber grip_dist;
    double gripper_distance;

	HexFT(const std::string& portStr)
    {   
        read_pub = nh.advertise<cam_motion::LoadCellMsg>("force_sensor", 1000);
        grip_dist = nh.subscribe("/upsampled_topic", 1000, &HexFT::callback, this);

        m_serialPort = open(portStr.c_str(), O_RDONLY);

        if (m_serialPort < 0) {
            std::cout << "Error " << errno << " from open: " << strerror(errno) << std::endl;
            std::cout << "This means that the sensor could not be found at " << portStr << std::endl;
            exit(-1);
        }
    }

	~HexFT()
    {
        close(m_serialPort);
    }

    void callback(const cam_motion::Force_msg::ConstPtr msg)
    {       
        gripper_distance = msg->normal_force.data;
        SensorData data;
        cam_motion::LoadCellMsg value;
        read(m_serialPort, reinterpret_cast<uint8_t*>(&data), sizeof(data));

        value.header.stamp = ros::Time::now();
        value.force_val.data.push_back(data.fX);
        value.force_val.data.push_back(data.fY);
        value.force_val.data.push_back(data.fZ);
        value.force_val.data.push_back(data.mX);
        value.force_val.data.push_back(data.mY);
        value.force_val.data.push_back(data.mZ);
        value.force_val.data.push_back(gripper_distance);
        read_pub.publish(value);
    }

private:
	int m_serialPort;
};

int main (int argc, char** argv){
    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);    
    ros::init(argc, argv, "serial_example_node");
    HexFT listen("/dev/ttyACM0");
    ros::spin();

    listen.~HexFT();

    return 0;
}