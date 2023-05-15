//----------------------------------------------------------------------------------//
#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
//#include <CytronMotorDriver.h>
#include "Config.h"
#include "Kinematics.h"
#include "Motor.h"

const char* ssid     = "Palof";
const char* password = "kakikuke";

Motor FL_motor(WheelA1, WheelA2, ENCA1, ENCA2);
Motor FR_motor(WheelB1, WheelB2, ENCB1, ENCB2);
Motor RL_motor(WheelC1, WheelC2, ENCC1, ENCC2); 
Motor RR_motor(WheelD1, WheelD2, ENCD1, ENCD2);
//CytronMD motor1(PWM_DIR, 10, A0);  // PWM 1 = Pin 10, DIR 1 = Pin A0.
//CytronMD motor2(PWM_DIR, 9, A1); // PWM 2 = Pin 9, DIR 2 = Pin A1.
Kinematics Robot(FL_motor, FR_motor, RL_motor, RR_motor);
void FLF_ISRA(); //Front-Left-Front
void FRF_ISRB(); //Front-Right-Front
void RLF_ISRC(); //Rear-Left-Front
void RRF_ISRD(); //Rear-Right-Front
void FLR_ISRA(); //Front-Right-Rear
void FRR_ISRB(); //Front-Right-Rear
void RLR_ISRC(); //Rear-Left-Rear
void RRR_ISRD(); //Rear-Right-Rear
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
void cmd_vel_callback(const geometry_msgs::Twist& twist);
std_msgs::Int16 flwheel_ticks_msg;
std_msgs::Int16 frwheel_ticks_msg;
std_msgs::Int16 rlwheel_ticks_msg;
std_msgs::Int16 rrwheel_ticks_msg;
std_msgs::Int16 flwheel_pwm_msg;
std_msgs::Int16 frwheel_pwm_msg;
std_msgs::Int16 rlwheel_pwm_msg;
std_msgs::Int16 rrwheel_pwm_msg;
ros::NodeHandle nh;
ros::Publisher flwheel_ticks_pub("frontleft_ticks", &flwheel_ticks_msg);
ros::Publisher frwheel_ticks_pub("frontright_ticks", &frwheel_ticks_msg);
ros::Publisher rlwheel_ticks_pub("rearleft_ticks", &rlwheel_ticks_msg);
ros::Publisher rrwheel_ticks_pub("rearright_ticks", &rrwheel_ticks_msg);
ros::Publisher flwheel_pwm_pub("frontleft_pwm", &flwheel_pwm_msg);
ros::Publisher frwheel_pwm_pub("frontright_pwm", &frwheel_pwm_msg);
ros::Publisher rlwheel_pwm_pub("rearleft_pwm", &rlwheel_pwm_msg);
ros::Publisher rrwheel_pwm_pub("rearright_pwm", &rrwheel_pwm_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);
unsigned long lastCmdVelReceived = 0;
float linearX_vel = 0, angularZ_vel = 0;
const float MIN_VELOCITY = 0.0;
const float MAX_VELOCITY = (2*PI*WHEEL_RADIUS*MAX_RPM)/(60*GEAR_REDUCTION);//0.728485253 m/s
//----------------------------------------------------------------------------------//

void setup(){

  IPAddress server(WiFi.localIP());
  const uint16_t serverPort = 11411;

  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) 
  { delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected - IP address:  ");
  Serial.println(WiFi.localIP());

  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCA2, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  pinMode(ENCB2, INPUT_PULLUP);
  pinMode(ENCC1, INPUT_PULLUP);
  pinMode(ENCC2, INPUT_PULLUP);
  pinMode(ENCD1, INPUT_PULLUP);
  pinMode(ENCD2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA1), FLF_ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB1), FRF_ISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCC1), RLF_ISRC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCD1), RRF_ISRD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA2), FLR_ISRA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB2), FRR_ISRB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCC2), RLR_ISRC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCD2), RRR_ISRD, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ESTOP), EMG_STOP, HIGH);
  nh.getHardware()->setConnection(server,serverPort);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(flwheel_ticks_pub);
  nh.advertise(frwheel_ticks_pub);
  nh.advertise(rlwheel_ticks_pub);
  nh.advertise(rrwheel_ticks_pub);
  nh.advertise(flwheel_pwm_pub);
  nh.advertise(frwheel_pwm_pub);
  nh.advertise(rlwheel_pwm_pub);
  nh.advertise(rrwheel_pwm_pub);
}

void loop(){
  nh.spinOnce();
  Robot.isRosConnected = nh.connected();

  //Stop the robot if not connected to ROS or there are no velocity command after some time
  if(!Robot.isRosConnected || (millis() - lastCmdVelReceived > CMD_VEL_TIMEOUT)){
    linearX_vel = 0;
    angularZ_vel = 0;
    Robot.Move(MIN_PWM, MIN_PWM, MIN_PWM, MIN_PWM);//stop motors
  }

  // Convert Linear X and Angular Z Velocity to PWM
  // Calculate left and right wheel velocity
  float  frontleft_vel = linearX_vel - angularZ_vel*(WHEEL_SEPARATION/2);
  float frontright_vel = linearX_vel + angularZ_vel*(WHEEL_SEPARATION/2);
  float  rearleft_vel = linearX_vel - angularZ_vel*(WHEEL_SEPARATION/2);
  float rearright_vel = linearX_vel + angularZ_vel*(WHEEL_SEPARATION/2);
  // Determine left and right direction using sign
  int  frontleft_dir = (frontleft_vel >0)? 1 : -1;
  int frontright_dir = (frontright_vel>0)? 1 : -1;
  int  rearleft_dir = (rearleft_vel >0)? 1 : -1;
  int rearright_dir = (rearright_vel>0)? 1 : -1;

  // Make sure calculated velocity is in range of min and max velocity before mapping to PWM
  if(fabs(frontleft_vel)>MAX_VELOCITY){
    frontleft_vel = frontleft_dir*MAX_VELOCITY;
  }
  else if(fabs(frontleft_vel)<MIN_VELOCITY){
    frontleft_vel = frontleft_dir*MIN_VELOCITY;
  }
  if(fabs(frontright_vel)>MAX_VELOCITY){
    frontright_vel = frontright_dir*MAX_VELOCITY;
  }
  else if(fabs(frontright_vel)<MIN_VELOCITY){
    frontright_vel = frontright_dir*MIN_VELOCITY;
  }
  if(fabs(rearleft_vel)>MAX_VELOCITY){
    rearleft_vel = rearleft_dir*MAX_VELOCITY;
  }
  else if(fabs(rearleft_vel)<MIN_VELOCITY){
    rearleft_vel = rearleft_dir*MIN_VELOCITY;
  }
  if(fabs(rearright_vel)>MAX_VELOCITY){
    rearright_vel = rearright_dir*MAX_VELOCITY;
  }
  else if(fabs(rearright_vel)<MIN_VELOCITY){
    rearright_vel = rearright_dir*MIN_VELOCITY;
  }
  

  // Map wheel velocity to PWM
  int  frontleft_pwm = round(mapFloat(fabs(frontleft_vel ), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM));
  int frontright_pwm = round(mapFloat(fabs(frontright_vel), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM));
  int  rearleft_pwm = round(mapFloat(fabs(rearleft_vel ), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM));
  int rearright_pwm = round(mapFloat(fabs(rearright_vel), MIN_VELOCITY, MAX_VELOCITY, MIN_PWM, MAX_PWM));
  
  // Actuate the motors
  flwheel_pwm_msg.data = frontleft_dir*frontleft_pwm;
  frwheel_pwm_msg.data = frontright_dir*frontright_pwm;
  rlwheel_pwm_msg.data = rearleft_dir*rearleft_pwm;
  rrwheel_pwm_msg.data = rearright_dir*rearright_pwm;
  Robot.Move(frontleft_dir*frontleft_pwm, frontright_dir*frontright_pwm, rearleft_dir*rearleft_pwm, rearright_dir*rearright_pwm);


  //Publishing data to ROS
  flwheel_ticks_pub.publish(&flwheel_ticks_msg);
  frwheel_ticks_pub.publish(&frwheel_ticks_msg);
  rlwheel_ticks_pub.publish(&rlwheel_ticks_msg);
  rrwheel_ticks_pub.publish(&rrwheel_ticks_msg);
  flwheel_pwm_pub.publish(&flwheel_pwm_msg);
  frwheel_pwm_pub.publish(&frwheel_pwm_msg);
  rlwheel_pwm_pub.publish(&rlwheel_pwm_msg);
  rrwheel_pwm_pub.publish(&rrwheel_pwm_msg);
  delay(200); //5Hz
}

////////////FUNCTION DEFINITIONS////////////////

void FLF_ISRA(){
  flwheel_ticks_msg.data = FL_motor.doEncoderA();
}

void FRF_ISRB(){
  frwheel_ticks_msg.data = FR_motor.doEncoderB();
}

void RLF_ISRC(){
  rlwheel_ticks_msg.data = RL_motor.doEncoderC();
}

void RRF_ISRD(){
  rrwheel_ticks_msg.data = RR_motor.doEncoderD();
}

void FLR_ISRA(){
  flwheel_ticks_msg.data = FL_motor.doEncoderA();
}

void FRR_ISRB(){
  frwheel_ticks_msg.data = FR_motor.doEncoderB();
}

void RLR_ISRC(){
  rlwheel_ticks_msg.data = RL_motor.doEncoderC();
}

void RRR_ISRD(){
  rrwheel_ticks_msg.data = RR_motor.doEncoderD();
}



float mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void cmd_vel_callback( const geometry_msgs::Twist& twist){
  linearX_vel = twist.linear.x;
  angularZ_vel = twist.angular.z;
  lastCmdVelReceived = millis();
}