/*
Tests PCB for Bone island wind alarm
Buzzer has short beep on startup
Selector switch is used to pick east or west LED to turn on.  When LED is on it rotates between green, yellow and red


Hardware: Arduino Pro-Mini 5v

*/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>

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

const byte SWITCH_IS_ON = LOW; 

enum ledDirection_t { EAST, WEST };

void ledGreen(ledDirection_t ledToTurnOn);
void ledRed(ledDirection_t ledToTurnOn);
void ledYellow(ledDirection_t ledToTurnOn);
void ledOff(ledDirection_t ledToTurnOn);


void setup()
{
 
  Serial.begin(9600);
  Serial.println("Testing Wind Alarm Board");
  
  pinMode(EAST_ALARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(WEST_ALARM_SWITCH_PIN, INPUT_PULLUP);
  pinMode(RESET_ALARM_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Turn LED's Off
  pinMode(EAST_ALARM_LED_PIN_R, OUTPUT);
  pinMode(EAST_ALARM_LED_PIN_G, OUTPUT);
  pinMode(EAST_ALARM_LED_PIN_B, OUTPUT);
  pinMode(WEST_ALARM_LED_PIN_R, OUTPUT);
  pinMode(WEST_ALARM_LED_PIN_G, OUTPUT);
  pinMode(WEST_ALARM_LED_PIN_B, OUTPUT);
  
  ledOff(EAST);
  ledOff(WEST);
  
  // test buzzer
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);
  
}  // end setup();


void loop()
{
 
  const byte LED_DELAY = 500;
  
  if (digitalRead(EAST_ALARM_SWITCH_PIN) == SWITCH_IS_ON )
  {
    ledGreen(EAST);
    delay(LED_DELAY);  
    ledYellow(EAST);
    delay(LED_DELAY);  
    ledRed(EAST);
    delay(LED_DELAY);  
    ledOff(EAST);
    delay(LED_DELAY);  
  }
  
  if (digitalRead(WEST_ALARM_SWITCH_PIN) == SWITCH_IS_ON )
  {
    ledGreen(WEST);
    delay(LED_DELAY);  
    ledYellow(WEST);
    delay(LED_DELAY);  
    ledRed(WEST);
    delay(LED_DELAY);  
    ledOff(WEST);
    delay(LED_DELAY);  
  }
  
  if (digitalRead(WEST_ALARM_SWITCH_PIN) != SWITCH_IS_ON  && digitalRead(EAST_ALARM_SWITCH_PIN) != SWITCH_IS_ON )
  {
    ledOff(WEST);
    ledOff(EAST);
    delay(LED_DELAY);
  }
  
  Serial.print(digitalRead(WEST_ALARM_SWITCH_PIN));
  Serial.print("\t");
  Serial.print(digitalRead(EAST_ALARM_SWITCH_PIN));
  Serial.print("\t");
  Serial.print(digitalRead(BUZZER_PIN));
  Serial.println();
  
  
    
  
} // end loop()


void ledOff(ledDirection_t ledToTurnOn)
{

  if (ledToTurnOn == EAST)
  {
  analogWrite(EAST_ALARM_LED_PIN_R, 255);
  analogWrite(EAST_ALARM_LED_PIN_G, 255);
  analogWrite(EAST_ALARM_LED_PIN_B, 255);
  }
  else
  {
  analogWrite(WEST_ALARM_LED_PIN_R, 255);
  analogWrite(WEST_ALARM_LED_PIN_G, 255);
  analogWrite(WEST_ALARM_LED_PIN_B, 255);
  }
} // ledOff

void ledGreen(ledDirection_t ledToTurnOn)
{

  if (ledToTurnOn == EAST)
  {
    analogWrite(EAST_ALARM_LED_PIN_R, 255);
    analogWrite(EAST_ALARM_LED_PIN_G,   0);
    analogWrite(EAST_ALARM_LED_PIN_B, 255);
  }
  else
  {
    analogWrite(WEST_ALARM_LED_PIN_R, 255);
    analogWrite(WEST_ALARM_LED_PIN_G,   0);
    analogWrite(WEST_ALARM_LED_PIN_B, 255);
  }
} // ledGreen



void ledRed(ledDirection_t ledToTurnOn)
{

  if (ledToTurnOn == EAST)
  {
    analogWrite(EAST_ALARM_LED_PIN_R,   0);
    analogWrite(EAST_ALARM_LED_PIN_G, 255);
    analogWrite(EAST_ALARM_LED_PIN_B, 255);
  }
  else
  {
    analogWrite(WEST_ALARM_LED_PIN_R,   0);
    analogWrite(WEST_ALARM_LED_PIN_G, 255);
    analogWrite(WEST_ALARM_LED_PIN_B, 255);
  }
} // ledRed


void ledYellow(ledDirection_t ledToTurnOn)
{

  if (digitalRead(EAST_ALARM_SWITCH_PIN) == SWITCH_IS_ON)
  {
    analogWrite(EAST_ALARM_LED_PIN_R,   0);
    analogWrite(EAST_ALARM_LED_PIN_G,   0);
    analogWrite(EAST_ALARM_LED_PIN_B, 255);
  }
  else
  {
    analogWrite(WEST_ALARM_LED_PIN_R,   0);
    analogWrite(WEST_ALARM_LED_PIN_G,   0);
    analogWrite(WEST_ALARM_LED_PIN_B, 255);
  }
} // ledYellow


