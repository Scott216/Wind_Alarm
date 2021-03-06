/* 

   RS-485: 9600 baud, 8-1-n  no handshaking

 03.1 026

848 46 48 32 50 55 52 13 10048484648325055521310048484648325055521310048

Line is terminated with CRLF (13 10)

Example: 
0 48 51 46 56 32 51 52 52 13 10 
converts to:
03.8 344
3.8 meters/sec
344 degrees

http://stackoverflow.com/questions/5697047/convert-serial-read-into-a-useable-string-using-arduino

Serial.buffer will hold 64 bytes total

Use software serial, not serial 1 for test

Only some pins can be used for SoftwareSerial Rx.  

*/

#define LINE_START 0

#include <Arduino.h>
#include <SoftwareSerial.h>  // http://arduino.cc/en/Reference/softwareSerial

const byte RS485_A_PIN =           13;  // Rx Pin
const byte RS485_B_PIN =           12;  // Tx Pin
SoftwareSerial windTracker(RS485_A_PIN, RS485_B_PIN);  // (Rx, Tx) 


void setup() 
{
  windTracker.begin(9600);     // opens serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  delay(4000);
//  while (!Serial) {;} // wait for serial monitor - Leonardo only

  Serial.println("RS-485 test");
  delay(500);
}

void loop()
{
  readData();
  
}



void readData()
{
  static uint32_t printoutTimer = millis();
  
  char windSpeed[5];
  char windDirection[4];
  byte b;
  if (windTracker.available() > 25) 
  { 
     
    b = windTracker.read();
    if ( b == LINE_START) 
    {
      for (byte s = 0; s < 4; s++)
      { windSpeed[s] = windTracker.read(); }
      windSpeed[4] = '\0';
      
      windTracker.read(); // read the space character
      
      for (byte d = 0; d < 3; d++)
      { windDirection[d] = windTracker.read(); }
      windDirection[3] = '\0';
      
      float windSpeedKnots = atof(windSpeed) * 1.94384;
      int iWindDirection = atoi(windDirection);
      
      if ( (long)(millis() - printoutTimer) > 0 )
      {
        Serial.print(millis()/1000);
        Serial.print("\t");
        Serial.print(windSpeedKnots);
        Serial.print("\t");
        Serial.println(iWindDirection);
        printoutTimer = millis() + 30000;  // add 30 second to printout timer
      }  
  }
    
  }
}
