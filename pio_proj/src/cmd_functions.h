#ifndef CMD_FUNCTIONS_H
#define CMD_FUNCTIONS_H

#include "misc_classes.h"

void move_translator(float speed)
{
  motor_translator.set_speed(speed);
}

void stop_translator()
{
  motor_translator.stop();
}

void move_motor_right(float speed)
{
  motor_right.set_speed(speed);
}

void stop_motor_right()
{
  motor_right.stop();
}

void move_motor_left(float speed)
{
  motor_left.set_speed(speed);
}

void stop_motor_left()
{
  motor_left.stop();
}

void stop_moving()
{
  stop_motor_right();
  stop_motor_left();
}

void light_on()
{
  digitalWrite(pinLight, LOW);
}

void light_off()
{
  digitalWrite(pinLight, HIGH);
}

void start_drilling()
{
  digitalWrite(pinDrill, LOW);
}

void stop_drilling()
{
  digitalWrite(pinDrill, HIGH);
}

void start_vacuum()
{
  digitalWrite(pinVacuum, LOW);
}

void stop_vacuum()
{
  digitalWrite(pinVacuum, HIGH);
}

void convert_speeds_and_move(double v, double w)
{
  //Originally coded by Joel. Change variables names to make sense
  double a = WHEEL_RADIUS / 2.0;
  double b = WHEEL_RADIUS / 2.0;
  double c = WHEEL_RADIUS / (2.0 * ROBOT_LENGTH);
  double d = -WHEEL_RADIUS / (2.0 * ROBOT_LENGTH);
  double e = v;
  double f = w;
  double determinant = a * d - b * c;

  if (determinant != 0)
  {
    double desiredLeftMotorSpeed = (e * d - b * f) / determinant;                                               // Vitesse désirée du moteur left (rad/s)
    double desiredRightMotorSpeed = (a * f - e * c) / determinant;                                               // Vitesse désirée du moteur right (rad/s)
    desiredLeftMotorSpeed = desiredLeftMotorSpeed * REDUCTION_RATIO_L * 60 / (2 * PI); // Vitesse désirée du moteur left (revMotor/min)
    desiredRightMotorSpeed = desiredRightMotorSpeed * REDUCTION_RATIO_R * 60 / (2 * PI); // Vitesse désirée du moteur right (revMotor/min)
    move_motor_right(-desiredLeftMotorSpeed); //opposite speed since symmetrical to left motor                             // On envoi la vitesse au moteur left (revMotor/min)
    move_motor_left(desiredRightMotorSpeed);                           // On envoi la vitesse au moteur right (revMotor/min)

    /*char tmp1[10];

    dtostrf(desiredLeftMotorSpeed, 1, 2, tmp1);

    nh.loginfo(tmp1);

    dtostrf(desiredRightMotorSpeed, 1, 2, tmp1);


    nh.loginfo(tmp1);*/
  }
}

void cmdCb(const geometry_msgs::Twist msg)
{
  convert_speeds_and_move(msg.linear.x * 100, msg.angular.z); //joel's code is in centimeters, but speed is published in meters
}

void lightStateCb(const std_msgs::Int8 msg)
{
  light_state = msg.data;
}

void translatorCb(const std_msgs::Float64 msg)
{

  if (msg.data == 0)
    stop_translator();
  else
    move_translator(-1 * msg.data * translator_speed);
}

void drillCb(const std_msgs::Bool msg)
{
  if (msg.data)
  {
    start_vacuum();
    start_drilling();
  }
  else
  {
    stop_drilling();
    stop_vacuum();
  }
}

void stop_all()
{
  stop_moving();
  stop_translator();
  stop_drilling();
}


void keepaliveCb(const std_msgs::Bool msg)
{
  keepalive_timer = 0;
}


#endif // CMD_FUNCTIONS_H
