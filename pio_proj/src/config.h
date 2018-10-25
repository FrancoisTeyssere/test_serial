#ifndef CONFIG_H
#define CONFIG_H

#define MAX_PWM 254
#define DEFAULT_TRANSLATOR_SPEED 200
#define MAX_TRANSLATOR_SPEED 600
#define REDUCTION_RATIO_L 100       // Rapport de réduction moteur left
#define REDUCTION_RATIO_R 100       // Rapport de réduction moteur right
#define REDUCTION_RATIO_T 5         // Rapport de réduction moteur up
#define WHEEL_RADIUS 7.5                       // Rayon des roues motrices (cm)
#define ROBOT_LENGTH 43.3                        // Longueur du robot (cm)
#define ROBOT_WIDTH 59.2                        // Largeur du robot (cm)

//Pin map

#define pinPot1 A7                   //Pannel Potentiometer pin
#define pinPot2 A6                   //Pannel Potentiometer pin

#define pinStart A5

#define pinDrill 3               // Pin numérique pour la ponceuse (sortie)
#define pinVacuum 15            // Pin numérique pour l'aspirateur (sortie)

#define pinDrillButton A1

#define pinLight 16
#define pinSettingButton 17
#define pinTranslatorUp 20
#define pinTranslatorDown 19

#define pinFWDLeft 4                 // Pin numérique pour le FWD du moteur left (sortie)
#define pinFWDRight 24               // Pin numérique pour le FWD du moteur right (sortie)
#define pinFWDUp 5                 // Pin numérique pour le FWD du moteur up (sortie)

#define pinREVLeft 6                 // Pin numérique pour le REV du moteur left (sortie)
#define pinREVRight 23                // Pin numérique pour le REV du moteur right (sortie)
#define pinREVUp 7                  // Pin numérique pour le REV du moteur up (sortie)

#define pinSTOPLeft 8                // Pin numérique pour le STOP du moteur left (sortie)
#define pinSTOPRight 22               // Pin numérique pour le STOP du moteur right (sortie)
#define pinSTOPUp 9                 // Pin numérique pour le STOP du moteur up (sortie)

#define pinSpeedLeft 10              // Pin numérique pour la vitesse du moteur left (sortie PWM)
#define pinSpeedRight 12             // Pin numérique pour la vitesse du moteur right (sortie PWM)
#define pinSpeedUp 11

#define pinTopSwitch A2 
#define pinBottomSwitch A3 
#define pinTorqueError A0
#define pinMidSwitch 18 //TO DEFINE

//Global variables

/*ros::NodeHandle nh;

std_msgs::Bool top_switch_msg;
std_msgs::Bool bottom_switch_msg;
std_msgs::Bool torque_error_msg;
std_msgs::Bool start_button_msg;*/




#endif // CONFIG_H
