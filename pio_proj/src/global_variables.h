#ifndef GLOBAL_VARIABLES_H
#define GLOBAL_VARIABLES_H

#include "elapsedMillis.h"


Button start_button(pinStart);
Button drill_button(pinDrillButton);
Button up_button(pinTranslatorUp);
Button down_button(pinTranslatorDown);
Button top_switch(pinTopSwitch);
Button bottom_switch(pinBottomSwitch);
TimeMeasuringButton setting_button(pinSettingButton);

Motor motor_right(pinFWDRight, pinREVRight, pinSTOPRight, pinSpeedRight);
Motor motor_left(pinFWDLeft, pinREVLeft, pinSTOPLeft, pinSpeedLeft);
Motor motor_translator(pinFWDUp, pinREVUp, pinSTOPUp, pinSpeedUp);

elapsedMillis light_timer = 0;
int light_delay = 2000;
int light_delay2 = 800;
bool current_light_state = false;

elapsedMillis keepalive_timer = 0;
int keepalive_delay = 2000;

elapsedMillis pub_timer = 0;
int pub_delay = 300;//only publish infos every 300ms

//elapsedMillis spin_timer = 0;

float translator_speed = DEFAULT_TRANSLATOR_SPEED;

int light_state = 0;//0 for off, 1 for on, 2 for blinking

#endif
