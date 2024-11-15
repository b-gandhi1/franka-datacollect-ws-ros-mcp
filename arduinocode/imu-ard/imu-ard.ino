#include <Arduino_LSM6DS3.h> // library for Arduino Nano 33 IOT
#include <stdlib.h>

// library for transmitting data through TCP
#include <SPI.h>
#include <WiFiNINA.h>


char ssid[] = "bhoomika-Latitude-5520";
char pass[] = "temporary";

char server[] = "143.167.51.103"; // IP address of the destination server
int port = 2055; // Port to connect (for HTTP it's usually 80)

WiFiClient client;

float x, y, z; // z unused
float degX = 0.0;
float degY = 0.0;

char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;

}

void setup() {
  Serial.begin(57600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB port only
  }

  // attempt to connect to WiFi network
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    delay(10000);
  }
  Serial.println("Connected to WiFi");

  if (!IMU.begin()) {
    Serial.println("Failed to initialise IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println("Hz");

  //  client.stop();
  // establish connection to server
  while (!client.connected())
  {
    Serial.println("Connecting to server...");
    if (client.connect(server, port))
    {
      Serial.println("Connected to server");
    }
    else
    {
      Serial.println("Connection failed");
    }
  }

}

void loop() {
  if (client.connected()) {
    if (IMU.accelerationAvailable())
    {
      IMU.readAcceleration(x, y, z);
      x = 100 * x;
      degX = map(x, -100, 97, -90, 90);
      y = 100 * y;
      degY = map(y, -100, 97, -90, 90);

      // Send your data here
      char buffer[32];
      dtostrf(degX, 5, 3, buffer); // Convert float to string with 5 digits and 3 decimal places
      strcat(buffer, ","); // Add a comma delimiter
      dtostrf(degY, 5, 3, buffer + strlen(buffer));
      client.println(buffer);
//      client.stop(); 
    }
    else
    {
      client.println(404.0404040);
      client.println(404.0404040);
    }
  }
  else {
    client.stop();
    Serial.println("Connection failed, retrying...");
    if (client.connect(server, port))
    {
      Serial.println("Connected to server");
    }
    else
    {
      Serial.println("Connection failed");
    }
  }

  delay(10);
}
