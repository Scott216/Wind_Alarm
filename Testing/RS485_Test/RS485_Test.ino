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

For Leonardo Serial1 is on pins pins D0 & D1
On the MAX485, Pin 4 (DI) goes to Arduino TX (D1) and Pin 1 (RO) goes to Arduino RX (D0)
RS-485 line A goes to MAX485 pin 6, line B goes to pin 7



*/

#define LINE_START 0

void setup() 
{
  Serial1.begin(9600);     // opens serial port, sets data rate to 9600 bps
  Serial.begin(9600);
  while (!Serial) {;} // wait for serial monitor - Leonardo only

  Serial.println("RS-485 test");
  delay(500);
}

void loop()
{
  readDataSerial1();
}



void readDataSerial1()
{

  char windSpeed[5];
  char windDirection[4];
  byte b;
  if (Serial1.available() > 25) 
  { 
     
    b = Serial1.read();
      if ( b == LINE_START) 
    {
      for (byte s = 0; s < 4; s++)
      { windSpeed[s] = Serial1.read(); }
      windSpeed[4] = '\0';
      
      Serial1.read(); // read the space character
      
      for (byte d = 0; d < 3; d++)
      { windDirection[d] = Serial1.read(); }
      windDirection[3] = '\0';
      
      float windSpeedKnots = atof(windSpeed) * 1.94384;
      int iWindDirection = atoi(windDirection);
      Serial.print(windSpeedKnots);
      Serial.print("\t");
      Serial.print(iWindDirection);
      Serial.print("\t");
      Serial.print(millis()/1000.0);
      Serial.println();
    }
  }
  
}
