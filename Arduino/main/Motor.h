#pragma once
class Motor{
  public:
    Motor(int, int, int, int);//driving motor with encoder
    void Rotate(int, int, int);
    long getEncoderPos();
    int getEncoderA();
    int getEncoderB();
    int getEncoderC();
    int getEncoderD();
    long doEncoderA();
    long doEncoderB();
    long doEncoderC();
    long doEncoderD();
  private:
    //motor pins  
    int D1;//pwm value
    int D2;//direction, low=forward, high=backward
    //encoder pins
    int ENA;
    int ENB;
    int ENC;
    int END;
    void initMotorPins();
    int protectOutput(int);
    void initEncoderPins();
    volatile long encoder_Pos = 0;
    bool drive; //true=driving , false=pantilt
};

Motor::Motor(int D1, int D2, int ENA, int ENB){
  this->D1 = D1;
  this->D2 = D2;
  this->drive = true;
  Motor::initMotorPins();
  this->ENA = ENA;
  this->ENB = ENB;
  Motor::initEncoderPins();
}

void Motor::initMotorPins(){
  pinMode(this->D1, OUTPUT);
  pinMode(this->D2, OUTPUT);
}

void Motor::Rotate(int pwm, int lower_lim=0, int upper_lim=0){
  // Actuate driving motor
  if(this->drive){
    // Set motor pwm value (inverted pwm values)
    analogWrite(this->D1, abs(pwm));
    // Enable (can move) or disable (cannot move) motor
    digitalWrite(this->D2, pwm!=0);
    // Set motor direction
    digitalWrite(this->D2, pwm<0);
    //digitalWrite(BRAKE,HIGH);
  }
}

int Motor::protectOutput(int val){
  // For safety reasons
  (val>150)? val = 150 : val;
  return val;
}

void Motor::initEncoderPins(){
  pinMode(this->ENA, INPUT_PULLUP);
  pinMode(this->ENB, INPUT_PULLUP);
  pinMode(this->ENC, INPUT_PULLUP);
  pinMode(this->END, INPUT_PULLUP);
}

long Motor::doEncoderA(){
  if(digitalRead(this->ENA)==HIGH){
    (digitalRead(this->ENB)==LOW)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  else{ 
    (digitalRead(this->ENB)==HIGH)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  return this->encoder_Pos;
}

long Motor::doEncoderB(){
  if(digitalRead(this->ENB)==HIGH){
    (digitalRead(this->ENA)==HIGH)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  else{ 
    (digitalRead(this->ENA)==LOW)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  return this->encoder_Pos;
}

long Motor::doEncoderC(){
  if(digitalRead(this->ENC)==HIGH){
    (digitalRead(this->END)==LOW)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  else{ 
    (digitalRead(this->END)==HIGH)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  return this->encoder_Pos;
}

long Motor::doEncoderD(){
  if(digitalRead(this->END)==HIGH){
    (digitalRead(this->ENC)==LOW)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  else{ 
    (digitalRead(this->ENC)==HIGH)? this->encoder_Pos++ : this->encoder_Pos--;
  }
  return this->encoder_Pos;
}


long Motor::getEncoderPos(){
  return this->encoder_Pos;
}

int Motor::getEncoderA(){
  return this->ENA;
}

int Motor::getEncoderB(){
  return this->ENB;
}

int Motor::getEncoderC(){
  return this->ENC;
}

int Motor::getEncoderD(){
  return this->END;
}