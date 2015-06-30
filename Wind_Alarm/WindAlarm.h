

#ifndef WindAlarm_h  // include guard

#define WindAlarm_h

#include <Arduino.h>

enum alarmStatus_t { ALARM_OK, ALARM_WARNING_SPEED, ALARM_WARNING_DIR, ALARM_WARNING_SPEED_DIR, ALARM_ON, ALARM_DISABLED, ALARM_HIGH_WIND, ALARM_WARNING_HIGH_WIND, OTHER_STATUS };
enum alarmCountdownType_t {WIND_SHIFT, HIGH_WIND};  // used in buzzer countdown timer to return either countdown to Wind shift alarm or to high wind alarm

const float LOW_PASS_FILTER_WIND = 0.01;  // used to smooth (average) wind data

class WindShiftAlarm
{
  public:
    WindShiftAlarm(int16_t direction_min, int16_t direction_max, int8_t wind_speed_threshold_knots, int8_t high_wind_speed_threshold_knots, uint16_t wind_time_threshold_seconds);
   ~WindShiftAlarm();
    void enableAlarm(bool activateAlarm);
    bool isAlarmEnabled();
    alarmStatus_t alarmStatus();
    bool isSpeedOk();
    bool isDirectionOk();
    void updateWind(float speed_now_knots, int16_t wind_direction_degrees);
    float getWindSpeed();
    int16_t getWindDirection();
    int16_t getAlarmCountdown(alarmCountdownType_t alarmCountdown);
    void resetTimer(); // used with the reset switch to reset the timer

  private:
    int16_t  _direction_min;  // If wind direction is inbetween _direction_min and _direction_max, then wind has shifted and alarm should sound if speed and time are met
    int16_t  _direction_max;
    float    _wind_speed_threshold_knots;
    float    _high_wind_speed_threshold_knots;
    float    _current_Speed_knots;  // current wind speed
    int16_t  _current_Direction;    // current wind direction in degrees
    bool     _AlarmIsEnabled;
    uint32_t _wind_time_threshold_seconds;    // Time (seconds) that wind has to be shifted before alarm will sound
    uint32_t _wind_Shift_Alarm_Time;          // Timestamp (ms) when speed and directin alarm conditions were met
    uint32_t _high_Wind_Alarm_Time;           // Timestamp (ms) when high wind speed exceeded threshold
    bool     _wind_Shift_Timer_Started_Flag;  // True when timer is running, used as a one shot to start timer as soon as both wind direction and speed are outside the threshold
    bool     _high_Wind_Timer_Started_Flag;   // True when timer is running for high wind alarm, used as a one shot to start the timer 
};

#endif // WindAlarm_h
