#include <Arduino_LSM6DS3.h> // library for Arduino Nano 33 IOT
#include <stdlib.h>

// library for transmitting data through TCP
#include <SPI.h>
#include <WiFiNINA.h>


char ssid[] = "bhoomika-Latitude-5520";
char pass[] = "temporary";

char server[] = "143.167.181.242"; // IP address of the destination server. NEEDS TO BE SAME AS IN SERVER.PY
//char my_hostname[] = "bgandhi1.shef.ac.uk"; // not currently working
//IPAddress server; // store IP address in this variable
int port = 2055; // Port to connect (for HTTP it's usually 80)

// get IP address:
//int err = WiFi.hostByName(my_hostname, server);


WiFiClient client;

float x, y, z; // z unused
float degX = 0.0;
float degY = 0.0;
String newval1, newval2;

char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;

}

void setup() {
  Serial.begin(57600);
  //  while (!Serial) {
  //    ; // Wait for serial port to connect. Needed for native USB port only
  //  }

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

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
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

  // IP address by hostname method. maybe this could replace above method??
//  if (err == 1) {
//    Serial.print("IP address: ");
//    Serial.println(server);
//  }
//  else {
//    Serial.print("Error code: ");
//    Serial.println(err);
//  }

}

void loop() {
  if (client.connected()) {
    if (IMU.gyroscopeAvailable())
    {
      IMU.readGyroscope(x, y, z);
      x = 100 * x;
      degX = map(x, -100, 97, -90, 90);
      Serial.print("X: ");
      Serial.print(degX);
      y = 100 * y;
      degY = map(y, -100, 97, -90, 90);
      Serial.print("  Y: ");
      Serial.println(degY);

      // Send your data here
      char buffer[32];
      dtostrf(degX, 5, 3, buffer); // Convert float to string with 5 digits and 3 decimal places
      strcat(buffer, ","); // Add a comma delimiter
      dtostrf(degY, 5, 3, buffer + strlen(buffer));
      newval1 = String(degX);
      newval2 = String(degY);
      client.println(buffer);
//      client.println(newval1+","+newval2);
//      Serial.println(newval1+","+newval2);
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
