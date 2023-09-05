// ROS ingration
#include <ros.h>
#include <std_msgs/Float32.h>

// ROS node handle
ros::NodeHandle nh;

// ROS publisher define
// publishing pump state (output) and pressure value (input) 
std_msgs::Float32 msg;
ros::Publisher pump_state("pump_state",&msg); // MODIFY
ros::Publisher pressure_val("pressure_val",&msg);

//PID constants
double kp = 2;
double ki = 5;
double kd = 1;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

// wired connections
#define HG7881_B_IA 9 // D9 --> Motor B Input A --> MOTOR B +
#define HG7881_B_IB 8 // D8 --> Motor B Input B --> MOTOR B -
#define HG7881_A_IA 2
#define HG7881_A_IB 3

// functional connections
#define VALVE_PWM HG7881_B_IA // Motor B PWM Speed
#define VALVE_DIR HG7881_B_IB // Motor B Direction
#define PUMP_PWM HG7881_A_IB
#define PUMP_DIR HG7881_A_IA

// air pressure sensor
#define air_pressure_pin A3 // pin A3 for sensing air pressure

// variables for air pressure sensor
float air_pressure_val;
float voltP;
float kPa;
// variable to publish
float pump_state_est;

void setup() {
  setPoint = 5; //5 kPa
  Serial.begin( 9600 );
  pinMode( VALVE_DIR, OUTPUT );
  pinMode( VALVE_PWM, OUTPUT );
  pinMode( PUMP_DIR, OUTPUT );
  pinMode( PUMP_PWM, OUTPUT );
  pinMode(air_pressure_pin, INPUT);
  digitalWrite( VALVE_DIR, HIGH );
  digitalWrite( VALVE_PWM, HIGH );
  digitalWrite( PUMP_DIR, LOW);
  digitalWrite( PUMP_PWM, LOW);
  
  // ROS setup
  nh.initNode();
  nh.advertise(pump_state);
  nh.advertise(pressure_val);
  
  delay(100);
  digitalWrite(PUMP_DIR, HIGH); //forward for filling in air
  
}

void loop() {
//  input = analogRead(A0); //read from rotary encoder connected to A0
//  output = computePID(input);
//  delay(100);
//  analogWrite(3, output); //control the motor based on PID value

  air_pressure_val = analogRead(air_pressure_pin);
  
  // convert ASCII to voltage
  voltP = (air_pressure_val*5)/1024;
  // convert to kPa
  kPa = voltP * (3/2) - (3/4); 
  // calculate pump state needed for measured kPa
  pump_state_est = computePID(kPa); 
  digitalWrite(PUMP_PWM, pump_state_est); // apply speed based on sensor feedback
  
  // Publish to ROS
//  pump_state_pub.data = pump_state;
//  pump_state.pump_state = pump_state_est;
//  kPa_pub.data = kPa;
//  pressure_val.pressure_val = kPa;
  pump_state.publish(pump_state_est);
  pressure_val.publish(kPa);

  nh.spinOnce();
}

double computePID(double inp) {
  currentTime = millis(); //get current time
  elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation

  error = Setpoint - inp;                                // determine error
  cumError += error * elapsedTime; // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative

  double out = kp * error + ki * cumError + kd * rateError;          //PID output

  lastError = error; //remember current error
  previousTime = currentTime; //remember current time

  if error <= 0.25
    {
      // close valve
      digitalWrite(VALVE_PWM, LOW);
      digitalWrite(VALVE_DIR, LOW); 
      digitalWrite(PUMP_DIR, LOW); // might not be needed... 
    }
  return out;                                        //have function return the PID output
}
