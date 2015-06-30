/*

Board: Arduino Pro-Mini 5v

Description:
Wind alarm for Bone Island.  Circuit will turn on a  buzzer and close a relay if the wind direction changes.  
In order for alarm to trigger, wind must change direction and be over a certian speed threshold and do this for a period of time.
Wind direction, speed and time thresholds are configured in WindShiftAlarm instance.
Circuit gets wind speed and direction from Young Wind Tracker via RS485 protocol. The protocol varies depending how old the Wind Tracker is.  
There are two wind alarm profiles, east dock and west dock.  
A 3-position selector switch is used to choose between East, West, and off.
There are two RGB LEDs, one for east alarm and one for west.  When the switch is moved toward east, for example, the LED will glow 
green to indicate east is the active alarm and everything is okay.  Once both the wind speed and direction go into the alarm range, the 
LED will turn yellow.  After it stays in that range for the specified number of seconds, LED will turn red and buzzer will sound.
There is another alaarm if the wind speed is very high, regardless of direction.  In this case the alarm will sound and both LEDs will turn red.


See Young_Wind_Tracker_RS-485_Protocol.txt file for info on the two protocols used


To do: 
Add bluetooth remote programming capability, maybe use sparkcore or bean instead of arduino pro
Add WDT
Add alert if lost if not getting RS485 data


I/O Requirements
6 PWM outputs for LEDs
1 digital out for buzzer
2 digital inputs for selector switch 
2 digital inputs for RS485 (RS485 will only work on certain pins)
1 digital input for reset switch - may not use this.


Wiring on back of Young Wind Tracker:
  Wire from sensor
   GND: Shield
   EXC: White
   WS: Red
   WD: Green
   REF: Black and Blue

  Wiring to PCB
    A - RS-485 A
    B - RS-485 B
    Ref - Gnd
        - +24v

Change Log
12/27/14  v0.02 - Changed LED code
02/23/15  v0.03 - Getting everything working
02/28/15  v0.04 - Added flag so once alarm is triggered, it can only be turned off by user
06/28/15  v0.05 - Wind display at Bone insland was different from the one I tested with, even though the model number is the same.
                  The format of the data is completely different.  Added getWindData_RMYT().  Changed printDebugData()
06/29/15 v0.06 - add buzzer chirp and high wind alarm - alarm goes off regardless of direction
06/30/15 v0.07 - increased knots to 7, flipped east/west settings, added global debug variables for speed

*/

#define VERSION "0.07"
#define PRINT_DEBUG

#include <Arduino.h>
#include "WindAlarm.h"
#include <SoftwareSerial.h>  // http://arduino.cc/en/Reference/softwareSerial
#include <HardwareSerial.h>

//#define PROTOCOL_NCAR
#define PROTOCOL_RMYT

// I/O
const byte RS485_A_PIN =           13; // Note: RS-485 doesn't work on most pins
const byte RS485_B_PIN =           12;
const byte EAST_ALARM_SWITCH_PIN =  4;
const byte WEST_ALARM_SWITCH_PIN =  2;
const byte RESET_ALARM_PIN =        7;  // Silences and resets alarm - might not use
const byte BUZZER_PIN =             8;  // Sounds buzzer
const byte EAST_ALARM_LED_PIN_R =   3;  // East alarm LED, Red, PWM
const byte EAST_ALARM_LED_PIN_B =   5;  // East alarm LED, Blue, PWM
const byte EAST_ALARM_LED_PIN_G =   6;  // East alarm LED, Green, PWM 
const byte WEST_ALARM_LED_PIN_R =   9;  // West alarm LED, Red, PWM
const byte WEST_ALARM_LED_PIN_B =  10;  // West alarm LED, Blue, PWM
const byte WEST_ALARM_LED_PIN_G =  11;  // West alarm LED, Green, PWM
// Unused: 0 & 1

const bool NO_BLINK_LED = false;
const bool BLINK_LED = true; 


// Create alarms (Min direction degrees, Max direction degrees, Speed threshold knots, time threshold seconds)
const int8_t   WIND_SHIFT_THRESHOLD_KNOTS = 7;  // Wind speed threshold for wind changing direction 
const int8_t   HIGH_WIND_THRESHOLD_KNOTS = 30;  // alarm goes off when sustained winds exceed this, regardless of direction
const uint16_t TIME_THRESHOLD_SEC =       600;  // Number of seconds that wind speed and direction must be maintained for alarm to turn on
WindShiftAlarm eastAlarm(180, 360, WIND_SHIFT_THRESHOLD_KNOTS, HIGH_WIND_THRESHOLD_KNOTS, TIME_THRESHOLD_SEC);
WindShiftAlarm westAlarm(0, 179, WIND_SHIFT_THRESHOLD_KNOTS, HIGH_WIND_THRESHOLD_KNOTS, TIME_THRESHOLD_SEC);

SoftwareSerial windTracker(RS485_A_PIN, RS485_B_PIN);

enum ledColor_t { RED, YELLOW, GREEN, OFF };
enum ledDirection_t { LED_NONE, LED_EAST, LED_WEST }; // used to select which LED to illuminate

// global debug variables
uint16_t g_debug_dispKnots = 0; // knots from display, bytes 7 & 8 
uint16_t g_debug_rawSpeed = 0; // raw wind speed, bytes 11 & 12


// function prototypes
void getWindData_NCAR();  // Wind data using NCAR protocol
void getWindData_RMYT();  // Wind data using RMYT protocol
void ledColor (ledColor_t color, ledDirection_t Led_to_Illuminate, bool blink_LED_Flag);
void buzzerChirp(uint16_t durationMs); 
void ledTestBlink();
void printDebugData(); 

void setup()
{
 
  Serial.begin(9600);
  Serial.print("Bone Island Wind Alarm ");
  Serial.println(VERSION);
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(EAST_ALARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(WEST_ALARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RESET_ALARM_PIN,       INPUT_PULLUP);  

  // Define RGB LED pins and set outputs so LEDs are off
  pinMode(EAST_ALARM_LED_PIN_R, OUTPUT);
  pinMode(EAST_ALARM_LED_PIN_G, OUTPUT);
  pinMode(EAST_ALARM_LED_PIN_B, OUTPUT);
  pinMode(WEST_ALARM_LED_PIN_R, OUTPUT);
  pinMode(WEST_ALARM_LED_PIN_G, OUTPUT);
  pinMode(WEST_ALARM_LED_PIN_B, OUTPUT);
  analogWrite(EAST_ALARM_LED_PIN_R, 255);
  analogWrite(EAST_ALARM_LED_PIN_G, 255);
  analogWrite(EAST_ALARM_LED_PIN_B, 255);
  analogWrite(WEST_ALARM_LED_PIN_R, 255);
  analogWrite(WEST_ALARM_LED_PIN_G, 255);
  analogWrite(WEST_ALARM_LED_PIN_B, 255);

  ledTestBlink();  // Cycle throuth LED colors
  buzzerChirp(5);  // quick buzzer chirp
  
  // set the data rate for the SoftwareSerial port to wind display
  windTracker.begin(9600);
  
}  // end setup();


void loop()
{  
  static bool keepAlarmOnFlag = false; // flag to keep buzzer on until user turns it off
  
  // Enable appropriate alarm.  Input is LOW if alarn is enabled
  // Switch is a 3-position switch: West-Off-East
  westAlarm.enableAlarm(!digitalRead(WEST_ALARM_SWITCH_PIN));
  eastAlarm.enableAlarm(!digitalRead(EAST_ALARM_SWITCH_PIN));


// Choose RS-485 protocol type
#ifdef PROTOCOL_NCAR  
  getWindData_NCAR(); // update wind speed and direction from Young WindTracker using NCAR protocol
#endif
#ifdef PROTOCOL_RMYT
  getWindData_RMYT(); // update wind speed and direction from Young WindTracker using RMYT Protocol
#endif

  // check alarm status
  alarmStatus_t status = ALARM_DISABLED;
  int16_t secondsToWindShiftBuzzerSound = 0;
  int16_t secondsToHighWindBuzzerSound = 0;
  ledDirection_t ActiveLed = LED_NONE; // ActiveLed determines if the east or west LED is active

  if( eastAlarm.isAlarmEnabled() ) 
  { 
    status = eastAlarm.alarmStatus(); 
    secondsToWindShiftBuzzerSound = eastAlarm.getAlarmCountdown(WIND_SHIFT);
    secondsToHighWindBuzzerSound = eastAlarm.getAlarmCountdown(HIGH_WIND);
    ActiveLed = LED_EAST;
  }
  else if( westAlarm.isAlarmEnabled() ) 
  { 
    status = westAlarm.alarmStatus(); 
    secondsToWindShiftBuzzerSound = westAlarm.getAlarmCountdown(WIND_SHIFT);
    secondsToHighWindBuzzerSound = westAlarm.getAlarmCountdown(HIGH_WIND);
    ActiveLed = LED_WEST;
  }
  else // both alarms are off
  { 
    status = ALARM_DISABLED; 
    ActiveLed = LED_NONE;
    keepAlarmOnFlag = false; // reset flag that keeps alarm on
  }
  
  static uint32_t buzzerChirpTimer = millis();  // used to chirp the buzzer every 5 seconds when alarm is about to go off
  switch ( status )
  {
    case ALARM_ON:
      // turn on buzzer and make LED red
      digitalWrite(BUZZER_PIN, HIGH);
      ledColor(RED, ActiveLed, NO_BLINK_LED);
      keepAlarmOnFlag = true;  // flag keeps alarm on until user turns it off
      break;
    case ALARM_WARNING_SPEED_DIR:
      if ( keepAlarmOnFlag == false )
      {
        digitalWrite(BUZZER_PIN, LOW);
        ledColor(YELLOW, ActiveLed, NO_BLINK_LED);
      
        // If alarm is close to going off, within 30 seconds, then chirp buzzer every 5 seconds
        if ( secondsToWindShiftBuzzerSound > 0 && secondsToWindShiftBuzzerSound <= 30 && (long)(millis() - buzzerChirpTimer) > 0 )
        {
          buzzerChirp(50);  // chirp buzzer for 50mS
          buzzerChirpTimer = millis() + 5000; // reset timer so buzzer will chirp again in 5 seconds
        }
      }
      break;
    case ALARM_WARNING_SPEED:
      if ( keepAlarmOnFlag == false )
      {
        digitalWrite(BUZZER_PIN, LOW);
        ledColor(GREEN, ActiveLed, NO_BLINK_LED);
      }
      break;
    case ALARM_WARNING_DIR:
      if ( keepAlarmOnFlag == false )
      {
        digitalWrite(BUZZER_PIN, LOW);
        ledColor(GREEN, ActiveLed, NO_BLINK_LED);
      }
      break;  
    case ALARM_OK:
      if ( keepAlarmOnFlag == false )
      {
        digitalWrite(BUZZER_PIN, LOW);
        ledColor(GREEN, ActiveLed, NO_BLINK_LED);
      }
      break;
    case ALARM_WARNING_HIGH_WIND:
      // Met high wind speed, but not for long enough time., don't do anything here
      // If alarm is close to going off, within 30 seconds, then chirp buzzer every 5 seconds
      // Chirp delay uses the same variables as wind shift alarm
      if ( keepAlarmOnFlag == false && secondsToHighWindBuzzerSound > 0 && secondsToHighWindBuzzerSound <= 30 && (long)(millis() - buzzerChirpTimer) > 0 )
      {
        buzzerChirp(50);  // chirp buzzer for 50mS
        delay(100);
        buzzerChirp(50);  // chirp buzzer for 50mS
        buzzerChirpTimer = millis() + 5000; // reset timer so buzzer will chirp again in 5 seconds
      }
      break;
    case ALARM_HIGH_WIND:  
      // turn on buzzer and make both LEDs red
      digitalWrite(BUZZER_PIN, HIGH);
      ledColor(RED, LED_EAST, NO_BLINK_LED);
      ledColor(RED, LED_WEST, NO_BLINK_LED);
      keepAlarmOnFlag = true;  // flag keeps alarm on until user turns it off
      break;
    case ALARM_DISABLED:
      digitalWrite(BUZZER_PIN, LOW);
      ledColor(OFF, LED_NONE, NO_BLINK_LED);
      break;
  }
  
  // Reset switch
  if ( digitalRead(RESET_ALARM_PIN) == LOW )
  { 
    eastAlarm.resetTimer(); 
    westAlarm.resetTimer(); 
    keepAlarmOnFlag = false;
  }
  #ifdef PRINT_DEBUG
    printDebugData();
  #endif
  
}  // end loop()


// Using software serial, read data from Young wind tracker display
// Data is NCAR Interactive Protocol: 
// &aaW: sss.s ddd<CR/LF>
// “aa” is the 09106 address in hex, 00-FF
// “sss.s” is speed
// “ddd” is direction in degrees
// <CR/LF> is the carriage return/line feed pair (ASCII 13 and 10).
void getWindData_NCAR()
{
  const byte LINE_START = 0;  // null character

  char windSpeed[5];
  char windDirection[4];
  byte b;
  if (windTracker.available() > 25)  // more then 25 characters in buffer
  { 
    b = windTracker.read();

    if ( b == LINE_START) 
    {
      // Get wind speed
      for (byte s = 0; s < 4; s++)
      { windSpeed[s] = windTracker.read(); }
      windSpeed[4] = '\0';
      
      windTracker.read(); // read the space character
      
      // Get the wind direction
      for (byte d = 0; d < 3; d++)
      { windDirection[d] = windTracker.read(); }
      windDirection[3] = '\0';
      
      float windSpeedKnots = atof(windSpeed) * 1.94384;
      uint16_t iWindDirection = atoi(windDirection);

      // Save wind speed and direction
      if(eastAlarm.isAlarmEnabled())
      { eastAlarm.updateWind(windSpeedKnots, iWindDirection); }
      if(westAlarm.isAlarmEnabled())
      { westAlarm.updateWind(windSpeedKnots, iWindDirection); }
      
    }   // end if LINE_START
  } // end if available
} // end getWindData_NCAR()

// Data on older Wind Tracker display is a 24 bytes staring with 
void getWindData_RMYT()
{
  byte b;
  static byte rawData[30];
  const byte LINE_START = 36;
  const byte SPEED_LSB =   8;
  const byte DIR_MSB =    13;
  const byte DIR_LSB =    14;

  if (windTracker.available() > 30)  // more then 30 characters in buffer
  { 
    rawData[0] = windTracker.read();

    if ( rawData[0] == LINE_START ) 
    {
      // check next 3 bytes to make sure we are at the start of the line
      rawData[1] = windTracker.read();  // Right display: 0=max wind speed, 1=direction
      rawData[2] = windTracker.read();  // Sensor and wind speed units: 12 = wind moninot with knots
      rawData[3] = windTracker.read();  // code for annunciator lights, 64 = knots
      if ( (rawData[1] == 0 || rawData[1] == 1) && rawData[2] == 12 &&rawData[3] == 64 )
      {
        // the 36 in rawData[0] is the start of the data, get the remaining bytes
        for ( int i = 4; i < 23; i++ )
        { rawData[i] = windTracker.read(); }

        float windSpeedKnots = rawData[SPEED_LSB];
        uint16_t iWindDirection = rawData[DIR_MSB] << 8;
        iWindDirection |=  rawData[DIR_LSB];

       // SRG Get wind speed for debugging
       g_debug_dispKnots = rawData[SPEED_LSB];
       g_debug_rawSpeed = rawData[11] << 8;
       g_debug_rawSpeed |= rawData[12];
       
        // Save wind speed and direction
        if(eastAlarm.isAlarmEnabled())
        { eastAlarm.updateWind(windSpeedKnots, iWindDirection); }
        if(westAlarm.isAlarmEnabled())
        { westAlarm.updateWind(windSpeedKnots, iWindDirection); }
      }
    }   // end if LINE_START
  } // end if available
} // end getWindData_RMYT()


// Set color of LED for enabled alarm
void ledColor (ledColor_t color, ledDirection_t Led_to_Illuminate, bool blink_LED_Flag)
{
  uint8_t Red_Pin, Grn_Pin, Blu_Pin;
  static uint32_t led_Blink_Timer = millis() + 250;
  static uint8_t ledState = 0; // used for blinking LEDs.  0 = LED on, 255 = LED off
  
  // Blink the LED
  if (blink_LED_Flag &&  ((long) (millis() - led_Blink_Timer) > 0 ) )
  {
    if ( ledState == 255 )
    { ledState = 0; }
    else
    { ledState = 255; }
    
    led_Blink_Timer = millis() + 200;
  }  

  // Setup LED pins so either the East LED or West LED is active
  switch ( Led_to_Illuminate )
  {
    case LED_WEST:
      Red_Pin = WEST_ALARM_LED_PIN_R;
      Grn_Pin = WEST_ALARM_LED_PIN_G;
      Blu_Pin = WEST_ALARM_LED_PIN_B;
      break;
    case LED_EAST:
      Red_Pin = EAST_ALARM_LED_PIN_R;
      Grn_Pin = EAST_ALARM_LED_PIN_G;
      Blu_Pin = EAST_ALARM_LED_PIN_B;
      break;
    default:
      break;
  } // end switch
  
  
  // RGB LEDs are common anode which means LED is off when output is high
  switch (color)
  {
    case RED:
      analogWrite(Red_Pin, ledState);
      analogWrite(Grn_Pin,      255);
      analogWrite(Blu_Pin,      255);
      break;
     case YELLOW:
      analogWrite(Red_Pin, ledState);
      analogWrite(Grn_Pin, ledState);
      analogWrite(Blu_Pin,      255);
      break;
     case GREEN:
      analogWrite(Red_Pin,      255);
      analogWrite(Grn_Pin, ledState);
      analogWrite(Blu_Pin,      255);
      break;
     case OFF: 
      analogWrite(EAST_ALARM_LED_PIN_R, 255);
      analogWrite(EAST_ALARM_LED_PIN_G, 255);
      analogWrite(EAST_ALARM_LED_PIN_B, 255);
      analogWrite(WEST_ALARM_LED_PIN_R, 255);
      analogWrite(WEST_ALARM_LED_PIN_G, 255);
      analogWrite(WEST_ALARM_LED_PIN_B, 255);
      break;
  }
}  // end ledColor()

// Chirp buzzer for a short time
void buzzerChirp(uint16_t durationMs)
{
  // Don't chirp for more then 1 second
  if (durationMs > 1000)
  { durationMs = 1000 ; }
  
  digitalWrite(BUZZER_PIN, HIGH);
  delay(durationMs);
  digitalWrite(BUZZER_PIN, LOW);
}  // end buzzerChirp()


// Both LEDs change color five times
void ledTestBlink()
{
  const uint16_t LED_DELAY = 400;
  
  for( byte i = 0; i < 3; i++ )
  {
    // Green
    analogWrite(EAST_ALARM_LED_PIN_R, 255);
    analogWrite(EAST_ALARM_LED_PIN_G,   0);
    analogWrite(EAST_ALARM_LED_PIN_B, 255);
    analogWrite(WEST_ALARM_LED_PIN_R, 255);
    analogWrite(WEST_ALARM_LED_PIN_G,   0);
    analogWrite(WEST_ALARM_LED_PIN_B, 255);
    delay(LED_DELAY);
    
   // Yellow
    analogWrite(EAST_ALARM_LED_PIN_R,   0);
    analogWrite(EAST_ALARM_LED_PIN_G,   0);
    analogWrite(EAST_ALARM_LED_PIN_B, 255);
    analogWrite(WEST_ALARM_LED_PIN_R,   0);
    analogWrite(WEST_ALARM_LED_PIN_G,   0);
    analogWrite(WEST_ALARM_LED_PIN_B, 255);
    delay(LED_DELAY);
  
    // Red
    analogWrite(EAST_ALARM_LED_PIN_R,   0);
    analogWrite(EAST_ALARM_LED_PIN_G, 255);
    analogWrite(EAST_ALARM_LED_PIN_B, 255);
    analogWrite(WEST_ALARM_LED_PIN_R,   0);
    analogWrite(WEST_ALARM_LED_PIN_G, 255);
    analogWrite(WEST_ALARM_LED_PIN_B, 255);
    delay(LED_DELAY);
  }

  // off
  analogWrite(EAST_ALARM_LED_PIN_R, 255);
  analogWrite(EAST_ALARM_LED_PIN_G, 255);
  analogWrite(EAST_ALARM_LED_PIN_B, 255);
  analogWrite(WEST_ALARM_LED_PIN_R, 255);
  analogWrite(WEST_ALARM_LED_PIN_G, 255);
  analogWrite(WEST_ALARM_LED_PIN_B, 255);
  delay(LED_DELAY);
}  // end ledTestBlink()


void printDebugData()
{
  static uint32_t printoutTimer = millis();
  static byte header_row = 0;
  if ( (long)( millis() - printoutTimer) > 0 )
  {
    if ( header_row++ == 20 )
    { 
      Serial.println("enabled\tspeed\tknots\traw\tdir\tspd OK\tdir OK\tbuzr\tcntdn1\tcntdn2\tstatus");
      header_row = 0;
    }
    if ( eastAlarm.isAlarmEnabled() ) 
    { 
      Serial.print("East"); 
      Serial.print("\t");
      Serial.print(eastAlarm.getWindSpeed());
      Serial.print("\t");
      Serial.print(g_debug_dispKnots);
      Serial.print("\t");
      Serial.print(g_debug_rawSpeed);
      Serial.print("\t");
      Serial.print(eastAlarm.getWindDirection());
      Serial.print("\t");
      Serial.print(eastAlarm.isSpeedOk());
      Serial.print("\t");
      Serial.print(eastAlarm.isDirectionOk());
      Serial.print("\t");
      Serial.print(digitalRead(BUZZER_PIN));
      Serial.print("\t");
      Serial.print(eastAlarm.getAlarmCountdown(WIND_SHIFT));
      Serial.print("\t");
      Serial.print(eastAlarm.getAlarmCountdown(HIGH_WIND));
      Serial.print("\t");
      switch ( eastAlarm.alarmStatus() )
      {
        case ALARM_OK:
          Serial.print("Status OK");
          break;
        case ALARM_WARNING_SPEED:
          Serial.print("Warning: Speed");
          break;
        case ALARM_WARNING_DIR:
          Serial.print("Warning: Direction");
          break;
        case ALARM_WARNING_SPEED_DIR:
          Serial.print("Warning: Speed and Direction");
          break;
        case ALARM_ON:
          Serial.print("Alarm On");
          break;
        case ALARM_WARNING_HIGH_WIND:
          Serial.print("Warning: High Wind");
          break;
        case ALARM_HIGH_WIND:
          Serial.print("Alarm high wind");
          break;
        case ALARM_DISABLED:
          Serial.print("Alarm Disabled");
          break;
        case OTHER_STATUS:
          Serial.print("Other Status");
          break;
        default: 
          Serial.print("Unknown Status");
          break;
      }
    }
    else if ( westAlarm.isAlarmEnabled() ) 
    { 
      Serial.print("West"); 
      Serial.print("\t");
      Serial.print(westAlarm.getWindSpeed());
      Serial.print("\t");
      Serial.print(g_debug_dispKnots);
      Serial.print("\t");
      Serial.print(g_debug_rawSpeed);
      Serial.print("\t");
      Serial.print(westAlarm.getWindDirection());
      Serial.print("\t");
      Serial.print(westAlarm.isSpeedOk());
      Serial.print("\t");
      Serial.print(westAlarm.isDirectionOk());
      Serial.print("\t");
      Serial.print(digitalRead(BUZZER_PIN));
      Serial.print("\t");
      Serial.print(westAlarm.getAlarmCountdown(WIND_SHIFT));
      Serial.print("\t");
      Serial.print(westAlarm.getAlarmCountdown(HIGH_WIND));
      Serial.print("\t");
      switch ( westAlarm.alarmStatus() )
      {
        case ALARM_OK:
          Serial.print("Status OK");
          break;
        case ALARM_WARNING_SPEED:
          Serial.print("Warning: Speed");
          break;
        case ALARM_WARNING_DIR:
          Serial.print("Warning: Direction");
          break;
        case ALARM_WARNING_SPEED_DIR:
          Serial.print("Warning: Speed and Direction");
          break;
        case ALARM_ON:
          Serial.print("Alarm On");
          break;
        case ALARM_WARNING_HIGH_WIND:
          Serial.print("Warning: High Wind");
          break;
        case ALARM_HIGH_WIND:
          Serial.print("Alarm high wind");
          break;
        case ALARM_DISABLED:
          Serial.print("Alarm Disabled");
          break;
        case OTHER_STATUS:
          Serial.print("Other Status");
          break;
        default: 
          Serial.print("Unknown Status");
          break;
      }
    }
    else
    { 
      Serial.print("OFF"); 
    }
    Serial.println();
    printoutTimer = millis() + 250; 
  }
}  // end printDebugData()

