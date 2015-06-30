/* 

RS-485: 9600 baud, 8-1-n  no handshaking

Read the raw RS-485 data.  The data is transmitted as ASCII code and the 
line is terminated with CRLF (13 10)

Example: 
0 48 51 46 56 32 51 52 52 13 10 
converts to:
48 = 0
51 = 3
46 = .
56 = 8
03.8 344
3.8 meters/sec
344 degrees

Data seems to be in a different format for display that mounted at Bone Island



http://stackoverflow.com/questions/5697047/convert-serial-read-into-a-useable-string-using-arduino

Serial.buffer will hold 64 bytes total

Use software serial, not serial 1 for test


*/

#define LINE_START 36

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

  Serial.println("RS-485 test");
  delay(500);
}

void loop()
{
  if ( millis() < 100000 )
  {  readData(); }  
}



// Read data and print out raw bytes
void readData()
{ 
  byte b;
  if (windTracker.available() > 0) 
  { 
     
    b = windTracker.read();
    if ( b == LINE_START) 
    { Serial.println(); }
    Serial.print(b);
    Serial.print(" ");
  }
}

// Read data and print out as ascii characters
void readDataAscii()
{ 
  byte b;
  if (windTracker.available() > 0) 
  { 
     
    b = windTracker.read();
    if ( b == LINE_START) 
    { Serial.println(); }
    Serial.print(char(b));
    Serial.print(" ");
  }
}

