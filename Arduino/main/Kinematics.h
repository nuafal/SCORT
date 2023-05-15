#pragma once
#include "Config.h"
#include "Motor.h"

class Kinematics{
  public:
    Kinematics(Motor, Motor, Motor, Motor); //4WD
    // Kinematics(Motor, Motor, Motor, Motor, LED, LED); //4WD with 2 LEDs
    void Move(int, int, int, int); //actuate all motors according to pwm/direction
    int isRosConnected;
  private:
    Motor FL_motor;
    Motor FR_motor;
    Motor RL_motor;
    Motor RR_motor;
};

Kinematics::Kinematics(Motor FL_motor, Motor FR_motor, Motor RL_motor, Motor RR_motor)
: FL_motor(FL_motor), FR_motor(FR_motor), RL_motor(RL_motor), RR_motor(RR_motor)
{

}

void Kinematics::Move(int fl_pwm, int fr_pwm, int rl_pwm, int rr_pwm){
  //add "-" sign to invert direction if needed
  this->FL_motor.Rotate(fl_pwm);
  this->FR_motor.Rotate(fr_pwm);
  this->RL_motor.Rotate(rl_pwm);
  this->RR_motor.Rotate(rr_pwm);
}
