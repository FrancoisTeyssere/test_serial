#ifndef LIGHT_MANAGER_H
#define LIGHT_MANAGER_H

#include "Arduino.h"
#include <openag_module.h>

class LightManager : public Module {
  public:
    // Constructor
    LightManager(int pin, bool is_active_low, int shutoff_ms);

    // Public functions
    uint8_t begin();
    uint8_t update();
    uint8_t set_cmd(int state);

  private:
    // Private variables
    int _pin;
    bool _is_active_low;
    int _shutoff_ms = 3000;
    uint32_t _last_cmd;
    int _state;
    int _light_timer;
    int _light_delay ;
    int _light_delay2;
    int _current_light_state;
};

#endif
