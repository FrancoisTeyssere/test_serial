#ifndef OPENAG_BINARY_SENSOR_H
#define OPENAG_BINARY_SENSOR_H

#include "Arduino.h"
#include <openag_module.h>

class BinarySensor : public Module {
  public:
    // Constructor
    BinarySensor(int pin, bool is_active_low);

    // Methods
    uint8_t begin();
    uint8_t update();
    bool get_is_on();
    bool wasPressed();

  private:
    // Private variables
    bool _is_on;
    bool _prev_state;
    int _pin;
    bool _is_active_low;
    uint32_t _time_of_last_reading;
    const static uint32_t _min_update_interval = 100;

    // Private methods
    void readData();
};

#endif
