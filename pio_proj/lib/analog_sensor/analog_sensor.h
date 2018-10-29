#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

#include "Arduino.h"
#include <openag_module.h>

class AnalogSensor : public Module {
  public:
    // Constructor
    AnalogSensor(int pin);

    // Methods
    uint8_t begin();
    uint8_t update();
    long getValue();

  private:
    // Private variables
    int _pin;
    uint32_t _time_of_last_reading;
    const static uint32_t _min_update_interval = 200;
    long _value;

    // Private methods
    void readData();
};

#endif
