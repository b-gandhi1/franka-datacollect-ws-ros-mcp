#include <Arduino_LSM6DS3.h> // library for Arduino Nano 33 IOT
#include <WiFiNINA.h>

// ROS ingration
//#include <ros.h>
//#include <std_msgs/Int16.h>

// wifi init: 
int status = WL_IDLE_STATUS;             // the Wi-Fi radio's status
int ledState = LOW;                       //ledState used to set the LED
unsigned long previousMillisInfo = 0;     //will store last time Wi-Fi information was updated
unsigned long previousMillisLED = 0;      // will store the last time LED was updated
const int intervalInfo = 5000;            // interval at which to update the board information

// ROS node handle
//ros::NodeHandle nh;

// ROS publisher define
//std_msgs::Int16 msg1;
//std_msgs::Int16 msg2;
//ros::Publisher rotX("rotX",&msg1);
//ros::Publisher rotY("rotY",&msg2);

float x, y, z; // z unused
float degX = 0;
float degY = 0;

void setup() {
  int baudrate = 57600;
//  int baudrate = 9600;
//  int baudrate = 34800;
//  int baudrate = 115200;
  
  Serial.begin(baudrate);
//  while(!Serial);
//  Serial.println("Started");
//
//  if (!IMU.begin()){
//    Serial.println("Failed to initialise IMU!");
//    while(1);
//  }
//  
//  Serial.print("Accelerometer sample rate = ");
//  Serial.print(IMU.accelerationSampleRate());
//  Serial.println("Hz");

  // set the LED as output
  pinMode(LED_BUILTIN, OUTPUT);

  // attempt to connect to Wi-Fi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
  
    // wait 10 seconds for connection:
    delay(10000);
  }
  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  Serial.println("---------------------------------------");

  // ROS setup
//  nh.initNode();
//  nh.getHardware()->setBaud(baudrate);
//  nh.advertise(rotX);
//  nh.advertise(rotY);
  
  delay(100);
  
}

void loop() {

  
  unsigned long currentMillisInfo = millis();
  // check if the time after the last update is bigger the interval
  if (currentMillisInfo - previousMillisInfo >= intervalInfo) {
    previousMillisInfo = currentMillisInfo;

    Serial.println("Board Information:");
    // print your board's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print your network's SSID:
    Serial.println();
    Serial.println("Network Information:");
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.println(rssi);
    Serial.println("---------------------------------------");
  }

  unsigned long currentMillisLED = millis();

  // measure the signal strength and convert it into a time interval
  int intervalLED = WiFi.RSSI() * -10;
 
  // check if the time after the last blink is bigger the interval 
  if (currentMillisLED - previousMillisLED >= intervalLED) {
    previousMillisLED = currentMillisLED;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    digitalWrite(LED_BUILTIN, ledState);
  }
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    x = 100 * x;
    degX = map(x,-100,97,-90,90);
    y = 100 * y;
    degY = map(y,-100,97,-90,90);

//    Serial.print("degX: ");
//    Serial.print(degX);
//    Serial.print("  degY: ");
//    Serial.println(degY);
    // Publish to ROS
//    msg1.data = degX;
//    rotX.publish(&msg1);
//    msg2.data = degY;
//    rotY.publish(&msg2);
//  
//    nh.spinOnce();
//    delay(10);
  }
  else {
    Serial.println("ERROR: IMU disconnected!");
    // Publish to ROS
//    msg1.data = 404;
//    rotX.publish(&msg1);
//    msg2.data = 404;
//    rotY.publish(&msg2);
//  
//    nh.spinOnce();
//    delay(10);
  }
  
}
