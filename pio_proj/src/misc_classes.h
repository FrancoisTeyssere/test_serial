#ifndef __MISC_CLASSES_H__
#define __MISC_CLASSES_H__

#include <elapsedMillis.h>
#include <EEPROM.h>


class Button
{

  public:

    Button(int pin) : _pin(pin) {}

    void update_state()
    {
      //nh.loginfo(digitalRead(_pin));
      _prev_state = _state;
      _state = !digitalRead(_pin);
    }

    const bool get_state() const
    {
      return _state;
    }

    const bool get_prev_state() const
    {
      return _prev_state;
    }

    const bool isMaintained() const
    {
      return _state;
    }

    const bool isPressed() const
    {
      return (_state && !_prev_state);
    }

    const bool wasPressed() const
    {
      return (!_state && _prev_state);
    }

  protected:

    int _pin;
    bool _state;
    bool _prev_state;
};


//******************************************************************//
//     SPECIAL CLASS OF BUTTON, MEASURING TIME PRESSED              //
//*****************************************************************//
class TimeMeasuringButton : public Button
{

  public:
    TimeMeasuringButton(int pin) : Button(pin), _isMeasuring(false) {}

    void update_state()
    {
      _prev_state = _state;
      _state = !digitalRead(_pin);

      //If button released, store elapsed time
      if (_isMeasuring && !isMaintained())
      {
        _isMeasuring = false;
        _timePressed = _timeElapsed;
      }

      //If button pressed, begin measuring time
      if (isPressed())
      {
        _isMeasuring = true;
        _timeElapsed = 0;
      }
    }

    const long& get_time_pressed() const
    {
      return _timePressed;
    }

  private:

    elapsedMillis _timeElapsed;
    long _timePressed;
    bool _isMeasuring;

};



class Motor
{
  public:
    Motor() {}
    Motor(int pinFWD, int pinREV, int pinSTOP, int pinSpeed) :
      _pinFWD(pinFWD),
      _pinREV(pinREV),
      _pinSTOP(pinSTOP),
      _pinSpeed(pinSpeed)
    {
      stop();
    }

    set_speed(float motor_speed)
    {
      float speed = motor_speed;
      float pwm_intensity = 0;

      if (motor_speed > 0)
      {
        speed = (speed + 28.254) / 0.864;   // Adjustement linéaire de la tension - max 3460
        digitalWrite(_pinFWD, HIGH);
        digitalWrite(_pinREV, LOW);
        digitalWrite(_pinSTOP, HIGH);
      }
      else if (motor_speed < 0)
      {
        speed = (speed - 28.254) / 0.864;   // Adjustement linéaire de la tension - max 3460
        digitalWrite(_pinFWD, LOW);
        digitalWrite(_pinREV, HIGH);
        digitalWrite(_pinSTOP, HIGH);
      }

      pwm_intensity = fabs(MAX_PWM * speed / 4000.0); // Puissance du PWM entre 0 et 255
      analogWrite(_pinSpeed, pwm_intensity);
    }


    stop()
    {
      digitalWrite(_pinFWD, HIGH);
      digitalWrite(_pinREV, HIGH);
      digitalWrite(_pinSTOP, HIGH);
      analogWrite(_pinSpeed, 0);
    }

  private:

    int _pinFWD;
    int _pinREV;
    int _pinSTOP;
    int _pinSpeed;
};


//*********************************************************//
//                       MISC FUNCTIONS                     //
//*********************************************************//

//This function will write a 4 uint8_t (32bit) long to the eeprom at
//the specified address to address + 3.

static void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 uint8_ts by using bitshift.
  //One = Most significant -> Four = Least significant uint8_t
  uint8_t four = (value & 0xFF);
  uint8_t three = ((value >> 8) & 0xFF);
  uint8_t two = ((value >> 16) & 0xFF);
  uint8_t one = ((value >> 24) & 0xFF);

  //Write the 4 uint8_ts into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}


static long EEPROMReadlong(long address)
{
  //Read the 4 uint8_ts from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

#endif
