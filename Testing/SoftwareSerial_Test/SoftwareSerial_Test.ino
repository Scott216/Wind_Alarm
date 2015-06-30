/* Receives from software serial
 
 The circuit: 
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)
 
 
 Not all pins on the Leonardo support change interrupts, 
 so only the following can be used for RX: 
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 
 
 */
#include <SoftwareSerial.h>

#define LINE_START 0

SoftwareSerial windtracker(10, 11); // RX, TX

void setup()  
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {;} // wait for serial monitor - Leonardo only
   
  Serial.println("RS-485 Test");

  // set the data rate for the SoftwareSerial port
  windtracker.begin(9600);
}

void loop() 
{
  readDataSoftwareSerial();
}


void readDataSoftwareSerial()
{

  char windSpeed[5];
  char windDirection[4];
  byte b;
  if (windtracker.available() > 25) 
  { 
     
    b = windtracker.read();
    if ( b == LINE_START) 
    {
      for (byte s = 0; s < 4; s++)
      { windSpeed[s] = windtracker.read(); }
      windSpeed[4] = '\0';
      
      windtracker.read(); // read the space character
      
      for (byte d = 0; d < 3; d++)
      { windDirection[d] = windtracker.read(); }
      windDirection[3] = '\0';
      
      float windSpeedKnots = atof(windSpeed) * 1.94384;
      int iWindDirection = atoi(windDirection);
      Serial.print(windSpeedKnots);
      Serial.print("\t");
      Serial.println(iWindDirection);
    }
  }
}
