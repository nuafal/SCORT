#pragma once
//DRIVING MOTOR PINS
/*#define A1 9 
#define A2 A1
#define LH_ENA 3
#define LH_ENB 11
#define RH_D1 12
#define RH_D2 A0     
#define RH_ENA 4
#define RH_ENB 2*/

#define WheelA1 13 //Front-Left
#define WheelA2 12
#define WheelB1 17 //Front-Right
#define WheelB2 18
#define WheelC1 1 //Rear-Left
#define WheelC2 2
#define WheelD1 26 //Rear-Right
#define WheelD2 25

#define ENCA1 3 //Front-Left
#define ENCA2 4 
#define ENCB1 34 //Front-Right
#define ENCB2 33 
#define ENCC1 14 //Rear-Left
#define ENCC2 15 
#define ENCD1 32 //Rear-Right
#define ENCD2 37 

//GENERAL CONSTANT //SPG10HP micro motor with encoder
#define CMD_VEL_TIMEOUT 500 //maximum time the robot will continue moving in the absence of a new command
#define PI 3.14159265359
#define WHEEL_DIAMETER 0.05 //cm
#define WHEEL_RADIUS 0.0025 
#define MAX_RPM 300
#define GEAR_REDUCTION 14 
#define TICKS_PER_METER 127.32 //ticks_per_meter = ticks_per_revolution(20) / (wheel_diameter * PI) 
#define WHEEL_SEPARATION 0.12
#define DISABLE_PWM 0 // disable motor driver
#define MIN_PWM 0 // 10% of 255 pwm = 26 pwm = 0 rpm
#define MAX_PWM 255 // 90% of 255 pwm = 229 pwm = 1000 rpm
#define STRAIGHT_PWM 75
#define TURN_PWM 65