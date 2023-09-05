// Found on: https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/Design/LowPass/LowPassFilter.ipynb 
// and https://github.com/curiores/ArduinoTutorials/blob/main/BasicFilters/ArduinoImplementations/LowPass/SimpleExamples/butterworth2.ino 

#define air_pressure_pin A0 // pin A3 for sensing air pressure

float air_pressure_val;
float voltP;
float kPa;

float x[] = {0,0,0};
float y[] = {0,0,0};
int k = 0;

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
  // convert to kPa
  kPa = voltP * (3/2) - (3/4); 

//  Serial.println(kPa);
  
  // x signal generate
  float t = micros()/1.0e06; // t in sec
  x[0] = voltP;
  
  // Compute the filtered signal
  // (second order Butterworth example)
  float b[] = {8.76503926e-05, 1.75300785e-04, 8.76503926e-05};
  float a[] = {1.97334504, -0.97369564};
  y[0] = a[0]*y[1] + a[1]*y[2] +
               b[0]*x[0] + b[1]*x[1] + b[2]*x[2];
//  Serial.println(kPa);
  delay(1);

  for(int i=1; i>=0; i--) // storing x and y past 2 values
  {
    x[i+1]=x[i];
    y[i+1]=y[i];
  }
  k++; // or k=k+1;

  Serial.println(y[0]); // this is the value that needs publishing in ROS. 

}
  
