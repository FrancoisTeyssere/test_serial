#include "light_manager.h".h"

LightManager::LightManager(int pin, bool is_active_low, int shutoff_ms):
  _light_delay(2000), _light_delay2(800), _light_timer(0), _current_light_state(0)

{
  _state = 0;
  _pin = pin;
  _is_active_low = is_active_low;
  _shutoff_ms = shutoff_ms;
  _last_cmd = 0;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

uint8_t LightManager::begin() {
  pinMode(_pin, OUTPUT);
  if (_is_active_low) {
    digitalWrite(_pin, HIGH);
  }
  else {
    digitalWrite(_pin, LOW);
  }
  return status_level;
}

uint8_t LightManager::update() {
  uint32_t curr_time = millis();
  if ((curr_time - _last_cmd) > _shutoff_ms) {
    if (_is_active_low) {
      digitalWrite(_pin, HIGH);
      _state = 0;
    }
    else {
      digitalWrite(_pin, LOW);
    }
  }
  else
  {
    switch(_state)
    {
    case 0:
      digitalWrite(_pin, HIGH);
      break;
    case 1:
      digitalWrite(_pin, LOW);
      break;
    case 2:
      if(millis() - _light_timer > _light_delay)
      {
        digitalWrite(_pin, !_current_light_state);
        _current_light_state = !_current_light_state;
        _light_timer = millis();
      }
      break;
    case 3:
      if(millis() - _light_timer > _light_delay2)
      {
        digitalWrite(_pin, !_current_light_state);
        _current_light_state = !_current_light_state;
        _light_timer = millis();
      }
      break;
    default:
      digitalWrite(_pin, HIGH);
      break;

    }
  }
  return status_level;
}

uint8_t LightManager::set_cmd(int state) {
  _last_cmd = millis();
  _state = state;
  /*if (cmd ^ _is_active_low) {
    digitalWrite(_pin, HIGH);
  }
  else {
    digitalWrite(_pin, LOW);
  }*/
  return status_level;
}
