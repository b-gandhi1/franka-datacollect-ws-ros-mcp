#define air_pressure_pin A0 // pin A3 for sensing air pressure

float air_pressure_val;
float voltP;
float kPa;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(air_pressure_pin, INPUT);
  Serial.println("Printing air pressure values ... ");
}

void loop() {
  // put your main code here, to run repeatedly:
  air_pressure_val = analogRead(air_pressure_pin);
//  Serial.println(air_pressure_val);
  
  // convert ASCII to voltage
  voltP = (air_pressure_val*5)/1024;

  Serial.println(voltP);
  // convert to kPa
  kPa = voltP * (3/2) - (3/4); 

//  Serial.println(kPa);
//  delay(1000);
  
}
