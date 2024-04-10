#include <Arduino_LSM6DS3.h> % library for Arduino Nano 33 IOT

// ROS ingration
#include <ros.h>
#include <std_msgs/Float32.h>

// ROS node handle
ros::NodeHandle nh;

// ROS publisher define
// publishing pump state (output) and pressure value (input) 
std_msgs::Float32 msg1;
std_msgs::Float32 msg2;
ros::Publisher rotX("rotX",&msg1);
ros::Publisher rotY("rotY",&msg2);

float x, y, z; // z unused
float degX = 0.0;
float degY = 0.0;

void setup() {
  int baudrate = 57600;
  Serial.begin(baudrate);
  while(!Serial);
  Serial.println("Started");

  if (!IMU.begin()){
    Serial.println("Failed to initialise IMU!");
    while(1);
  }
  
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");
  
  // ROS setup
  nh.initNode();
  nh.getHardware()->setBaud(baudrate);
  nh.advertise(rotX);
  nh.advertise(rotY);
  
  delay(100);
  
}

void loop() {

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    x = 100 * x;
    degX = map(x,-100,97,-90,90);
    y = 100 * y;
    degY = map(y,-100,97,-90,90);
    // Publish to ROS
    msg1.data = degX;
    rotX.publish(&msg1);
    msg2.data = degY;
    rotY.publish(&msg2);
  
    nh.spinOnce();
    delay(10);
  }
  else {
    Serial.println("ERROR: IMU disconnected!");
    // Publish to ROS
    msg1.data = 404.404;
    rotX.publish(&msg1);
    msg2.data = 404.404;
    rotY.publish(&msg2);
  
    nh.spinOnce();
    delay(10);
  }
  
}
