// wired connections
#define HG7881_B_IA 9 // D9 --> Motor B Input A --> MOTOR B +
#define HG7881_B_IB 8 // D8 --> Motor B Input B --> MOTOR B -
#define HG7881_A_IA 2
#define HG7881_A_IB 3 

#define pressurePin A0 // pin 11 for pressure sensor

// functional connections
#define VALVE_PWM HG7881_B_IA // Motor B PWM Speed
#define VALVE_DIR HG7881_B_IB // Motor B Direction
#define PUMP_PWM HG7881_A_IB
#define PUMP_DIR HG7881_A_IA 

// define variables
int pressureV = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode( VALVE_DIR, OUTPUT );
  pinMode( VALVE_PWM, OUTPUT );
  pinMode( PUMP_DIR, OUTPUT );
  pinMode( PUMP_PWM, OUTPUT );
  pinMode(pressurePin, INPUT);
  digitalWrite( VALVE_DIR, LOW );
  digitalWrite( VALVE_PWM, HIGH );
  digitalWrite( PUMP_DIR, LOW);
  digitalWrite( PUMP_PWM, LOW);

}

void loop() {
  pressureV = analogRead(pressurePin);
  Serial.println(pressureV);
  delay(500);
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
    Serial.print( "?" );
    while( !Serial.available() )
      ; // LOOP...
    c = Serial.read();
    // execute the menu option based on the character recieved
    switch( c )
    {
      case '1': // 1) Forward      
        Serial.println( "Remove air..." );
        // always stop motors briefly before abrupt changes
        digitalWrite( PUMP_DIR, LOW );
        digitalWrite( PUMP_PWM, LOW );
        delay( 500 );
        // set the motor speed and direction
        digitalWrite( PUMP_DIR, HIGH ); // direction = forward
        digitalWrite( PUMP_PWM, HIGH ); // PWM speed = slow
        isValidInput = true;
        break;   

     case '2': // 2) Reverse
        Serial.println( "Fill air..." );
        // always stop motors briefly before abrupt changes
        digitalWrite( PUMP_DIR, LOW );
        digitalWrite( PUMP_PWM, LOW );
        delay( 500 );
        // set the motor speed and direction
        digitalWrite( PUMP_DIR, LOW ); // direction = reverse
        digitalWrite( PUMP_PWM, HIGH ); 
        isValidInput = true;
        break;
         
      case '3': // 3) Soft stop (preferred)
        Serial.println( "Soft stop (coast)..." );
        digitalWrite( PUMP_DIR, LOW );
        digitalWrite( PUMP_PWM, LOW );
        isValidInput = true;
        break;            
         
      default:
        // wrong character! display the menu again!
        isValidInput = false;
        Serial.println("Wrong character! Select again.");
        break;
    }
  } while( isValidInput == true );

}
