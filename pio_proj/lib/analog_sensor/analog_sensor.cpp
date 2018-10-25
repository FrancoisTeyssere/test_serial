/**
 *  \file openag_binary_sensor.cpp
 *  \brief Binary Sensor for OpenAg brain.
 */
/***************************************************

 ****************************************************/
#include "analog_sensor.h"

AnalogSensor::AnalogSensor(int pin) {
  _pin = pin;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

uint8_t AnalogSensor::begin() {
  pinMode(_pin, INPUT);
  _time_of_last_reading = 0;
  return status_level;
}

uint8_t AnalogSensor::update() {
  if (millis() - _time_of_last_reading > _min_update_interval) {
    readData();
    _time_of_last_reading = millis();
  }
  return status_level;
}


void AnalogSensor::readData() {
  _value = analogRead(_pin);
}

long AnalogSensor::getValue()
{
  return _value;
}
