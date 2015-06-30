/*

RS-485: 9600 baud, 8-1-n  no handshaking

Read the raw RS-485 data.  The data is transmitted as ASCII code and the

Trying to decode protocol.  I think it's RMYT, which is Young's proprietery protocol



http://stackoverflow.com/questions/5697047/convert-serial-read-into-a-useable-string-using-arduino

Serial.buffer will hold 64 bytes total

Use software serial, not serial 1 for test


*/

#define LINE_START 36
const byte BUZZER_PIN =  8;  // Sounds buzzer

#include <Arduino.h>
#include <SoftwareSerial.h>  // http://arduino.cc/en/Reference/softwareSerial

const byte RS485_A_PIN =           13;  // Rx Pin
const byte RS485_B_PIN =           12;  // Tx Pin
SoftwareSerial windTracker(RS485_A_PIN, RS485_B_PIN);  // (Rx, Tx)


void setup()
{
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  windTracker.begin(9600);     // opens serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  delay(4000);

  Serial.println("RS-485 test for Young Disiplay");
  delay(500);
}

void loop()
{
    readData();
}



// Read data and print out raw bytes
void readData()
{
  byte b;
  static byte rawData[30];
  if (windTracker.available() > 30)
  {

    b = windTracker.read();
    if ( b == LINE_START)
    {
      // check next 3 bytes
      byte b1 = windTracker.read();
      byte b2 = windTracker.read();
      byte b3 = windTracker.read();
      if ( b1 == 0 && b2 == 12 && b3 == 64 )
      {
        for ( int i = 0; i < 20; i++ )
        {
          rawData[i] = windTracker.read();
//          Serial.print(rawData[i]);
//          Serial.print(" ");
        }
        
        // Print data if direction > 255
        if ( rawData[10] > 245 || rawData[1] == 1 )
        {
          for ( int i = 0; i < 20; i++ )
          {
//            Serial.print(rawData[i]);
//            Serial.print(" ");
          }
//           Serial.println();
        }

        Serial.print(rawData[4]);
        Serial.print("\t");
        uint16_t direction = rawData[9] << 8;
        direction |=  rawData[10]; 
        Serial.print(direction);
        Serial.println();
      }
    }  // end line start
  } // end available

}

