// ROS ingration
//#include <ros.h>
//#include <std_msgs/Float32.h>
//#include <std_msgs/String.h>

// ROS node handle
//ros::NodeHandle nh;

// ROS publisher define
// publishing pump state (output) and pressure value (input) 
//std_msgs::String msg1;
//std_msgs::Float32 msg2;
//ros::Publisher chatter1("pump_state",&msg1);
//ros::Publisher chatter2("pressure_val",&msg2);

String pump_state = "";

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

#define pressure_pin A0 // A0 for pressure sensor 

double pressure_val;

void setup()
{
  int baudrate = 9600;
  Serial.begin(baudrate);
  pinMode(pressure_pin, INPUT);
  pinMode(VALVE_DIR, OUTPUT);
  pinMode(VALVE_PWM, OUTPUT);
  pinMode(PUMP_DIR, OUTPUT);
  pinMode(PUMP_PWM, OUTPUT);
  digitalWrite(VALVE_DIR, HIGH);
  digitalWrite(VALVE_PWM, HIGH);
  digitalWrite(PUMP_DIR, LOW);
  digitalWrite(PUMP_PWM, LOW);

  // ROS setup
//  nh.initNode();
//  nh.getHardware()->setBaud(baudrate);
//  nh.advertise(chatter1);
//  nh.advertise(chatter2);
}

void loop() {
  boolean isValidInput;
  // draw a menu on the serial port
  Serial.println( "-----------------------------" );
  Serial.println( "MENU:" );
  Serial.println( "1) Remove Air" );
  Serial.println( "2) Fill Air" );
  Serial.println( "3) STOP" );
  Serial.println( "-----------------------------" );

  do
  {
    byte c;
    // get the next character from the serial port
    Serial.print( "Ready. Enter your option." );
    while ( !Serial.available() )
      ; // LOOP...
    c = Serial.read();
    // execute the menu option based on the character recieved
    switch ( c )
    {
      case '1': // 1) Forward
        Serial.println( "Remove air..." );
        pump_state = "Remove air";
        // always stop motors briefly before abrupt changes
        digitalWrite( PUMP_DIR, LOW );
        digitalWrite( PUMP_PWM, LOW );
        delay( 500 );
        // set the motor speed and direction
        digitalWrite( PUMP_DIR, HIGH ); // direction = forward
        digitalWrite( PUMP_PWM, HIGH ); // PWM speed = slow
        isValidInput = true;

        // read and print pressure value
        pressure_val = analogRead(pressure_pin);
        Serial.println(pressure_val);
        
        break;

      case '2': // 2) Reverse
        Serial.println( "Fill air..." );
        pump_state = "Fill air";
        // always stop motors briefly before abrupt changes
        digitalWrite( PUMP_DIR, LOW );
        digitalWrite( PUMP_PWM, LOW );
        delay( 500 );
        // set the motor speed and direction
        digitalWrite( PUMP_DIR, LOW ); // direction = reverse
        digitalWrite( PUMP_PWM, HIGH );
        isValidInput = true;
        
        // read and print pressure value
        pressure_val = analogRead(pressure_pin);
        Serial.println(pressure_val);
        
        break;

      case '3': // 3) Soft stop (preferred)
        Serial.println( "Soft stop (coast)..." );
        pump_state = "Stop";
        digitalWrite( PUMP_DIR, LOW );
        digitalWrite( PUMP_PWM, LOW );
        isValidInput = true;
        
        // read and print pressure value
        pressure_val = analogRead(pressure_pin);
        Serial.println(pressure_val);
        
        break;

      default:
        // wrong character! display the menu again!
        isValidInput = false;
        break;
    }
    // publish values to ROS
//    msg1.data = pump_state;
//    chatter1.publish(&msg1);
//    
//    nh.spinOnce();
//    delay(10);
  } while ( isValidInput == true );

}
