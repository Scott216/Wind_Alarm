

#include "WindAlarm.h"

// constructor for alarm instance with alarm settings
WindShiftAlarm::WindShiftAlarm(int16_t direction_min, int16_t direction_max, int8_t wind_speed_threshold_knots, int8_t high_wind_speed_threshold_knots, uint16_t wind_time_threshold_seconds)
{
  _direction_min = direction_min;
  _direction_max = direction_max;
  _wind_speed_threshold_knots = wind_speed_threshold_knots;   
  _high_wind_speed_threshold_knots = high_wind_speed_threshold_knots; 
  _wind_time_threshold_seconds = wind_time_threshold_seconds;
  _AlarmIsEnabled = false;
  _wind_Shift_Timer_Started_Flag = false;
  _high_Wind_Timer_Started_Flag = false;
  _wind_Shift_Alarm_Time = millis();
  _high_Wind_Alarm_Time = millis();
  
}  // end constructor


// destructor
WindShiftAlarm::~WindShiftAlarm()
{
  _direction_min = 0;
  _direction_max = 0; 
  _wind_speed_threshold_knots = 0;
  _wind_time_threshold_seconds = 0;
  _current_Speed_knots = 0;
  _current_Direction = 0; 
  _AlarmIsEnabled = false;
  _wind_Shift_Timer_Started_Flag = false;
  _high_Wind_Timer_Started_Flag = false; 
}  // end destructor


// Enable or disable an alarm
void WindShiftAlarm::enableAlarm(bool activateAlarm)
{

  // If alarm is changing state from disabled to enabled, then reset threshold timer
  if ( _AlarmIsEnabled == false && activateAlarm == true)
  { resetTimer(); }  
  
   _AlarmIsEnabled = activateAlarm;  // save state of alarm
  
  // If disabled, the set speed and direction to -1 so it's clear it's not real data
  if ( _AlarmIsEnabled == false )
  {  // alarm is disabled
    _current_Speed_knots = -1;  
    _current_Direction =   -1;
  }

}  // end enableAlarm()


bool WindShiftAlarm::isAlarmEnabled()
{
  return _AlarmIsEnabled;
}  // end isAlarmEnabled()


// Returns status of alarm
//    ALARM_OK = wind speed and direction are okay
//    ALARM_WARNING_SPEED = wind speed exceeds threshold, wind direction is okay
//    ALARM_WARNING_DIR = wind direction is outside of threshold, wind speed is okay
//    ALARM_WARNING_SPEED_DIR = wind & speed in alarm range, but not long enough to trigger buzzer
//    ALARM_ON = alarm is on, wind speed and direction are both outside the threshold for longer then the time allowed
//    ALARM_HIGH_WIND = Wind is very high, any direction
//    ALARM_WARNING_HIGH_WIND = wind is high, any direction, hasn't resched time threshold yet
//    ALARM_DISABLED = alarm disabled
//    OTHER_STATUS = Not sure of status, shouldn't ever be this
alarmStatus_t WindShiftAlarm::alarmStatus()
{
  if ( _AlarmIsEnabled )
  {
    if ( _wind_Shift_Timer_Started_Flag )  
    {
       // Speed and direction are in alarm limits, check timer
      if ( (long) (millis() - _wind_Shift_Alarm_Time ) > ( _wind_time_threshold_seconds * 1000UL ) )
      { return ALARM_ON; } // Time threshold has passed, return alarm on
      else
      { return ALARM_WARNING_SPEED_DIR; } // Time threshold not reached yet, return warning
    }
    else if ( _high_Wind_Timer_Started_Flag ) 
    {
       // high wind speed condition, check timer
      if ( (long) (millis() - _high_Wind_Alarm_Time ) > ( _wind_time_threshold_seconds * 1000UL ) )
      { return ALARM_HIGH_WIND; } // Time threshold has passed, return alarm on
      else
      { return ALARM_WARNING_HIGH_WIND; } // high wind time threshold not reached yet, return warning
    }
    else if ( !isSpeedOk() )
    { return ALARM_WARNING_SPEED; }
    else if ( !isDirectionOk() )
    { return ALARM_WARNING_DIR; } 
    else if (isSpeedOk() && isDirectionOk() )
    { // Wind direction or speed or both are in acceptable range
      resetTimer();
      return ALARM_OK; 
    } 
    else
    { return OTHER_STATUS; }
    
  }
  else // alarm disabled
  { return ALARM_DISABLED; }; // alarm is disabled
  
}  // end alarmStatus()


bool WindShiftAlarm::isSpeedOk()
{
  if ( _current_Speed_knots < _wind_speed_threshold_knots )
  { return true; }
  else
  { return false; }
}  // end isSpeedOk()


bool WindShiftAlarm::isDirectionOk()
{
  if ( _current_Direction >= _direction_min && _current_Direction <= _direction_max )
  { return true; }
  else
  { return false; }   
}  // end isDirectionOk()


// Updates wind speed and direction and checks them against alarm settings
void WindShiftAlarm::updateWind(float speed_now_knots, int16_t wind_direction_degrees )
{  
  _current_Speed_knots = (speed_now_knots * LOW_PASS_FILTER_WIND) + (1.0 - LOW_PASS_FILTER_WIND) * _current_Speed_knots;
  _current_Direction = wind_direction_degrees;

  // Check the speed and direction against settings.  If both meet the alarm limits, then start the timer
  if ( !isSpeedOk() && !isDirectionOk() && _AlarmIsEnabled )
  {
    // Wind speed and direction meet the alarm limits
    if (_wind_Shift_Timer_Started_Flag == false)
    { 
      _wind_Shift_Alarm_Time = millis();   // Timestamp when wind alarm speed and direction conditions were met
      _wind_Shift_Timer_Started_Flag = true;    // Prevents _wind_Shift_Alarm_Time from resetting every time this function is called
    }
  } 
  else
  { _wind_Shift_Timer_Started_Flag = false; } // Wind alarm conditions have not been met, reset timer flag
  
  //  If high wind speed, then start high wind timer
  if ( getWindSpeed() >= _high_wind_speed_threshold_knots && _AlarmIsEnabled )
  {
   // High wind speed met alarm limits
    if (_high_Wind_Timer_Started_Flag == false)
    { 
      _high_Wind_Alarm_Time = millis();        // Timestamp when high wind alarm conditions were met
      _high_Wind_Timer_Started_Flag = true;    // Prevents _high_Wind_Alarm_Time from resetting every time this function is called
    }
  }
  else
  { _high_Wind_Timer_Started_Flag = false; } // High wind conditions not met, reset flag
} // end updateWind()


// return wind speed in knots
float WindShiftAlarm::getWindSpeed()
{
  return _current_Speed_knots;
}  // end getWindSpeed()


// Return wind direction in degrees
int16_t WindShiftAlarm::getWindDirection()
{
  return _current_Direction;
}  // end getWindDirection()


// Returns number of seconds left before alarm will go off
int16_t WindShiftAlarm::getAlarmCountdown( alarmCountdownType_t alarmCountdown ) 
{
  int16_t secondsLeftToSoundBuzzer = 0; 

  if (alarmCountdown == HIGH_WIND )
  {
     secondsLeftToSoundBuzzer = _wind_time_threshold_seconds - (( millis() - _high_Wind_Alarm_Time) / 1000 );
    
    if( alarmStatus() == ALARM_WARNING_HIGH_WIND && secondsLeftToSoundBuzzer > 0 )
    { return secondsLeftToSoundBuzzer; }  
    else
    { return 0; }
  } 
  else // WIND_SHIFT 
  {
     secondsLeftToSoundBuzzer = _wind_time_threshold_seconds - (( millis() - _wind_Shift_Alarm_Time) / 1000 );
    
    if( alarmStatus() == ALARM_WARNING_SPEED_DIR && secondsLeftToSoundBuzzer > 0 )
    { return secondsLeftToSoundBuzzer; }  
    else
    { return 0; }
  } 
} // end getAlarmCountdown()


// reset the alarm
void WindShiftAlarm::resetTimer()
{
  _wind_Shift_Alarm_Time = millis();
  _high_Wind_Alarm_Time = millis();
  _wind_Shift_Timer_Started_Flag = false;
  _high_Wind_Timer_Started_Flag = false; 
}  // end resetTimer()


