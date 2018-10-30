#include <openag_binary_sensor.h>

#include <openag_pwm_actuator.h>
#include <openag_binary_actuator.h>
#include <analog_sensor.h>
#include <ble2_motor.h>
#include <light_manager.h>

#include "config.h"
//#include "misc_classes.h"
//#include "global_variables.h"
//#include "cmd_functions.h"

#define BUFFER_SIZE 200


// Sensor Instances
BinarySensor torque_error(pinTorqueError, true);
BinarySensor top_switch(pinTopSwitch, true);
BinarySensor bottom_switch(pinBottomSwitch, true);
BinarySensor mid_switch(pinMidSwitch, true);
BinarySensor start_button(pinStart, true);
BinarySensor drill_button(pinDrillButton, true);
BinarySensor setting_button(pinSettingButton, true);
BinarySensor translator_up(pinTranslatorUp, true);
BinarySensor translator_down(pinTranslatorDown, true);

AnalogSensor pot1(pinPot1);
AnalogSensor pot2(pinPot2);

// Actuator Instances. Sorted by pin number.
// The types are kind of irrelevant and I think we don't even need some of these.

BinaryActuator drill(pinDrill, true, 3000);
BinaryActuator vacuum(pinVacuum, true, 3000);
LightManager light(pinLight, true, 3000);


BLE2Motor motor_left(pinSpeedLeft, pinFWDLeft, pinREVLeft, pinSTOPLeft, 3000);
BLE2Motor motor_right(pinSpeedRight, pinFWDRight, pinREVRight, pinSTOPRight, 3000);
BLE2Motor motor_up(pinSpeedUp, pinFWDUp, pinREVUp, pinSTOPUp, 3000);




// Message string
String message = "";
char input_buff[BUFFER_SIZE] = {0};
bool stringComplete = false;
const int COMMAND_LENGTH = 7; // status + num_actuators
String msg_state = "";
int nb_char = 0;

// Timing constants
uint32_t delayMs = 50; //ms
uint32_t prev_time = millis();

void split(String messages, String* splitMessages,  char delimiter=',');
void actuatorLoop();
void sensorLoop();

// #region Arduino Events
void setup() {
  Serial.begin(115200);
  while(!Serial){
    // wait for serial port to connect, needed for USB
  }
  message.reserve(200);

  // Begin sensors
  beginModule(torque_error, "Torque Error #1");
  beginModule(top_switch, "Top Switch #1");
  beginModule(bottom_switch, "Bottom Switch #1");
  beginModule(mid_switch, "Mid Switch #1");
  beginModule(start_button, "Start Button #1");
  beginModule(drill_button, "Dril Button");
  beginModule(setting_button, "Setting Button");
  beginModule(translator_up, "Translator Up");
  beginModule(translator_down, "Translator Down");

  beginModule(pot1, "Potentiometer 1");
  beginModule(pot2, "Potentiometer 2");


  // Begin Actuators
  beginModule(drill, "Drill");
  beginModule(vacuum, "Vacuum");
  beginModule(light, "Pump 3, pH Up");

  beginModule(motor_left, "Motor Left");
  beginModule(motor_right, "Motor Right");
  beginModule(motor_up, "Motor Up");

  pinMode(52, OUTPUT);
  digitalWrite(52, LOW);

}

void loop() {


  // Throttle the Arduino since the python node can't keep up
  // and the serial buffer overflows. We do this without blocking.
  if(millis() - prev_time > delayMs){

  prev_time = millis();

  //run sensor loop first to use actualised value in actuator loop
  sensorLoop();
  actuatorLoop();
  }
}

// Runs inbetween loop()s, just takes any input serial to a string buffer.
// Runs as realtime as possible since loop has no delay() calls. (It shouldn't!)

void empty_buffer()
{
  for(int i=0;i<BUFFER_SIZE;i++)

  input_buff[i] = 0;
}
void serialEvent() {
  if(stringComplete){
    message = "";
    empty_buffer();
    stringComplete = false;
    //msg_state = 0;
    nb_char = 0;

  }
  long TIMEOUT = 10000;
  unsigned long timeout = millis() + TIMEOUT;
  while (Serial.available() /*&& (millis() - timeout > 0)*/) {
    digitalWrite(13, HIGH);

    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    input_buff[nb_char] = inChar;
    //message = Serial.readString();
    nb_char++;

    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
      //msg_state = message[0];
      message = String((char*)input_buff);
      //Serial.println(message);
      digitalWrite(13, LOW);

      return;
    }
  }
  /*if(millis() - timeout < 0)
  {

    message = ""; //if we timed out, reset message
    //msg_state = 2;
  }*/
}
// #endregion

bool manage_translator(float speed)
{
  if(translator_up.get_is_on())
  {
    motor_up.set_cmd(-1.5*DEFAULT_TRANSLATOR_SPEED);
    return true;
  }
  else if (translator_down.get_is_on())
  {
    motor_up.set_cmd(pot1.getValue()*MAX_TRANSLATOR_SPEED / 1024);
    return true;
  }
  else if(translator_up.wasPressed() || translator_down.wasPressed())
  {
    motor_up.stop();
    return false;
  }
  else //if nothing happened on command pannel, process message
  {
    motor_up.set_cmd(speed*pot1.getValue()*MAX_TRANSLATOR_SPEED / 1024);

    return (speed != 0);
  }
}

bool manage_drill(bool cmd)
{
  if(drill_button.get_is_on())
  {
    drill.set_cmd(true);
    vacuum.set_cmd(true);
    return true;
  }
  else if(drill_button.wasPressed())
  {
    drill.set_cmd(false);
    vacuum.set_cmd(false);
    return false;
  }
  else
  {
    drill.set_cmd(cmd);
    vacuum.set_cmd(cmd);
    return cmd;
  }
}

void actuatorLoop(){
  // If serial message, actuate based on it.
  if(stringComplete){
    //msg_state = 1;
    String splitMessages[COMMAND_LENGTH];

    for(int i = 0; i < COMMAND_LENGTH; i++){
      splitMessages[0] = "";
      //Serial.print(message[i]);
    }
    split(message, splitMessages);

    // We've already used this message
    message = "";
    stringComplete = false;
    nb_char = 0;
    empty_buffer();

    if(splitMessages[0] != "0"){
      return;
    }
    //msg_state = splitMessages[4];
    light.set_cmd(str2bool(splitMessages[3]));
    motor_left.set_cmd(splitMessages[4].toFloat());

    motor_right.set_cmd(splitMessages[5].toFloat());
    //allows to handle manual command directly from arduino, in case of link mismatch
    manage_drill(str2bool(splitMessages[1]));
    manage_translator(splitMessages[6].toFloat());

  }
  //else
    //msg_state = 2;

  // Run the update loop
  bool allActuatorSuccess = true;

  allActuatorSuccess = updateModule(drill, "Drill") && allActuatorSuccess;
  allActuatorSuccess = updateModule(vacuum, "Vacuum") && allActuatorSuccess;
  allActuatorSuccess = updateModule(light, "Light") && allActuatorSuccess;
  allActuatorSuccess = updateModule(motor_left, "Motor Left") && allActuatorSuccess;
  allActuatorSuccess = updateModule(motor_right, "Motor Right") && allActuatorSuccess;
  allActuatorSuccess = updateModule(motor_up, "Motor up") && allActuatorSuccess;

  if(!allActuatorSuccess){
    return;
  }
}

void sensorLoop(){
  bool allSensorSuccess = true;

  // Run Update on all sensors
  allSensorSuccess = updateModule(torque_error, "torque error") && allSensorSuccess;
  allSensorSuccess = updateModule(top_switch, "top switch") && allSensorSuccess;
  allSensorSuccess = updateModule(bottom_switch, "bottom switch") && allSensorSuccess;
  allSensorSuccess = updateModule(mid_switch, "mid switch") && allSensorSuccess;
  allSensorSuccess = updateModule(start_button, "start button") && allSensorSuccess;
  allSensorSuccess = updateModule(drill_button, "drill button") && allSensorSuccess;
  allSensorSuccess = updateModule(setting_button, "setting button") && allSensorSuccess;
  allSensorSuccess = updateModule(translator_up, "translator up") && allSensorSuccess;
  allSensorSuccess = updateModule(translator_down, "translator down") && allSensorSuccess;
  allSensorSuccess = updateModule(pot1, "Potentiometer 1") && allSensorSuccess;
  allSensorSuccess = updateModule(pot2, "Potentiometer 2") && allSensorSuccess;


  if(!allSensorSuccess){
    return;
  }

  // Prints the data in CSV format via serial.
  // Columns: status,hum,temp,co2
  Serial.print(OK);                                             Serial.print(',');
  Serial.print(torque_error.get_is_on());                    Serial.print(',');
  Serial.print(top_switch.get_is_on());                    Serial.print(',');
  Serial.print(bottom_switch.get_is_on());                    Serial.print(',');
  Serial.print(mid_switch.get_is_on());                    Serial.print(',');
  Serial.print(start_button.wasPressed());                    Serial.print(',');
  Serial.print(drill_button.get_is_on());                    Serial.print(',');
  Serial.print(setting_button.get_is_on());                    Serial.print(',');
  Serial.print(translator_up.get_is_on());                    Serial.print(',');
  Serial.print(translator_down.get_is_on());                    Serial.print(',');
  Serial.print(pot1.getValue());                    Serial.print(',');
  Serial.print(pot2.getValue()); Serial.print('\n');
  // https://www.arduino.cc/en/serial/flush
  // Wait until done writing.
  Serial.flush();
}

// #region helpers
// C is disgusting and I hate it deeply...
void split(String messages, String* splitMessages,  char delimiter){
  int indexOfComma = 0;
  for(int i = 0; messages.indexOf(delimiter, indexOfComma) > 0; i++){
    int nextIndex = messages.indexOf(delimiter, indexOfComma+1);
    String nextMessage;

    // The first message doesn't have an initial comma, so account for that.
    if(indexOfComma == 0){
      indexOfComma = -1;
    }
    if(nextIndex == -1){
      nextMessage = messages.substring(indexOfComma+1);
    }else{
      nextMessage = messages.substring(indexOfComma+1, nextIndex);
    }
    splitMessages[i] = nextMessage;
    indexOfComma = nextIndex;
  }

}

bool beginModule(Module &module, String name){
  bool status = module.begin() == OK;
  if(!status){
    Serial.print(module.status_level); Serial.print(',');
    Serial.print(name);  Serial.print(',');
    Serial.print(module.status_code);  Serial.print(',');
    Serial.print(module.status_msg);   Serial.print('\n');
    Serial.flush();
  }
  return status;
}

bool updateModule(Module &module, String name){
  bool status = module.update() == OK;
  if(!status){
    Serial.print(module.status_level); Serial.print(',');
    Serial.print(name);  Serial.print(',');
    Serial.print(module.status_code);
    // Serial.print(',');
    // Serial.print(module.status_msg);
    Serial.print('\n');
    Serial.flush();
  }
  return status;
}

bool any(bool *all){
  int length = sizeof(all)/sizeof(all[0]);
  for(int i=0; i < length; i++){
    if(all[i]){
      return true;
    }
  }
  return false;
}

bool str2bool(String str){
  return (str == "True");
}
// #endregion
