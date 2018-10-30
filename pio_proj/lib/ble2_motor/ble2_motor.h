#ifndef BLE2_MOTOR_H
#define BLE2_MOTOR_H

#include "Arduino.h"
#include <openag_module.h>

class BLE2Motor : public Module {
  public:
    // Constructor
    BLE2Motor(int pin_speed, int pin_fwd, int pin_rev, int pin_stop, int shutoff_ms);

    // Public functions
    uint8_t begin();
    uint8_t update();
    uint8_t set_cmd(float motor_speed);
    uint8_t stop();

  private:
    // Private variables
    int _pin_speed;
    int _pin_fwd;
    int _pin_rev;
    int _pin_stop;
    int _shutoff_ms = 3000;
    uint32_t _last_cmd;
};

#endif
