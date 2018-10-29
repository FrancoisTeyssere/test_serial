#include "ble2_motor.h"

BLE2Motor::BLE2Motor(int pin_speed, int pin_fwd, int pin_rev, int pin_stop,int shutoff_ms) {
  _pin_speed = pin_speed;
  _pin_fwd = pin_fwd;
  _pin_rev = pin_rev;
  _pin_stop = pin_stop;

  _shutoff_ms = shutoff_ms;
  _last_cmd = 0;
  status_level = OK;
  status_code = CODE_OK;
  status_msg = "";
}

uint8_t BLE2Motor::begin() {
  pinMode(_pin_speed, OUTPUT);
  pinMode(_pin_fwd, OUTPUT);
  pinMode(_pin_rev, OUTPUT);
  pinMode(_pin_stop, OUTPUT);
  digitalWrite(_pin_fwd, HIGH);
  digitalWrite(_pin_rev, HIGH);
  digitalWrite(_pin_stop, HIGH);
  analogWrite(_pin_speed, 0);

  return status_level;
}

uint8_t BLE2Motor::update() {
  uint32_t curr_time = millis();
  if ((curr_time - _last_cmd) > _shutoff_ms) {
    digitalWrite(_pin_fwd, HIGH);
    digitalWrite(_pin_rev, HIGH);
    digitalWrite(_pin_stop, HIGH);
    analogWrite(_pin_speed, 0);
  }
  return status_level;
}

uint8_t BLE2Motor::stop()
{
return set_cmd(0);
}

uint8_t BLE2Motor::set_cmd(float motor_speed) {
  _last_cmd = millis();
  float speed = motor_speed;
  float pwm_intensity = 0;

  if (motor_speed > 0)
  {
    speed = (speed + 28.254) / 0.864;   // Adjustement linéaire de la tension - max 3460
    digitalWrite(_pin_fwd, HIGH);
    digitalWrite(_pin_rev, LOW);
    digitalWrite(_pin_stop, HIGH);
  }
  else if (motor_speed < 0)
  {
    speed = (speed - 28.254) / 0.864;   // Adjustement linéaire de la tension - max 3460
    digitalWrite(_pin_fwd, LOW);
    digitalWrite(_pin_rev, HIGH);
    digitalWrite(_pin_stop, HIGH);
  }
  else //if speed ==0
  {
    digitalWrite(_pin_fwd, HIGH);
    digitalWrite(_pin_rev, HIGH);
    digitalWrite(_pin_stop, HIGH);
  }

  pwm_intensity = fabs(255 * speed / 4000.0); // Puissance du PWM entre 0 et 255
  analogWrite(_pin_speed, pwm_intensity);
  return status_level;
}
