// #define USE_TEENSY_ENCODER

#include <ros.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>


#include <Arduino.h>
#include <Encoder.h>
#include <CytronMotorDriver.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
//
// //Serial Comms
#define BAUD_RATE 115200
#define NUM_PARAMS 3
#define NUM_MOTORS 3
#define PID_FREQ 60
#define PWM_FREQ 20000
#define MAX_SPEED 255
#define SPEED_PRCNT 0.75


#define RANGE_MAX 1023 //Max val of pot
#define RANGE_MIN 0 //Min Val of pot
#define MIDPOINT 0

#define ROTATION 4752

#define ENC1_A    0
#define ENC1_B    1

#define ENC2_A    2
#define ENC2_B    3

#define ENC3_A    4
#define ENC3_B    5

#define MOT1_PWM1 8
#define MOT1_PWM2 9
#define MOT1_EN   16

#define MOT2_PWM1 10
#define MOT2_PWM2 11
#define MOT2_EN   17

#define MOT3_PWM1 12
#define MOT3_PWM2 13
#define MOT3_EN   18

Encoder enc1(ENC1_A,ENC1_B);
Encoder enc2(ENC2_A,ENC2_B);
Encoder enc3(ENC3_A,ENC3_B);

CytronMD motor1(PWM_DIR, MOT1_PWM1, MOT1_EN);
CytronMD motor2(PWM_DIR, MOT2_PWM1, MOT2_EN);
CytronMD motor3(PWM_DIR, MOT3_PWM1, MOT3_EN);

//PID Function Prototype
float computePID(int setpoint, int state, int channel,float _deadband);
void limit_speed_percentage(float max_speed, float percent, int* output);

//Global Vals for PID control
int setpoints[NUM_PARAMS] = {MIDPOINT,MIDPOINT,MIDPOINT}, setpoints_old[NUM_PARAMS] = {MIDPOINT,MIDPOINT,MIDPOINT};

//PID Vars
float kP[3] = {1,1,1}; //0.125
float kI[3] = {0.0,0.0,0.0}; //0.001
float kD[3] = {0.1,0.1,0.1};

float deadband = 50.0;

float volatile currentTime[NUM_MOTORS], previousTime[NUM_MOTORS], elapsedTime[NUM_MOTORS];
float volatile error[NUM_MOTORS]={0,0,0}, cumError[NUM_MOTORS]={0,0,0}, rateError[NUM_MOTORS]={0,0,0}, lastError[NUM_MOTORS]={0,0,0};
void set_goal( const std_msgs::Int32MultiArray& goal_msg);




ros::NodeHandle  nh;

std_msgs::Int32MultiArray pos_fb;

ros::Publisher pos_pub("OWB_pos_error", &pos_fb);
ros::Subscriber<std_msgs::Int32MultiArray> goal_sub("OWB_goal_pos", &set_goal);



void setup() {
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
  nh.advertise(pos_pub);
  nh.subscribe(goal_sub);


  pos_fb.data_length = 3;

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
  analogWriteFrequency(MOT1_PWM1, PWM_FREQ);
  analogWriteFrequency(MOT2_PWM1, PWM_FREQ);
  analogWriteFrequency(MOT3_PWM1, PWM_FREQ);
  
  enc1.write(0);
  enc2.write(0);
  enc3.write(0);
  
  delay(500);
  digitalWrite(13,LOW);
}

void loop() {

      nh.spinOnce();

      // Compute PID and send to motors.
      int out1 = computePID(setpoints[0], enc1.read(), 0, deadband);
      int out2 = computePID(setpoints[1], enc2.read(), 1, deadband);
      int out3 = computePID(setpoints[2], enc3.read(), 2, deadband);
      
      limit_speed_percentage(MAX_SPEED, SPEED_PRCNT, &out1);
      limit_speed_percentage(MAX_SPEED, SPEED_PRCNT, &out2);
      limit_speed_percentage(MAX_SPEED, SPEED_PRCNT, &out3);
      
      motor1.setSpeed(out1);
      motor2.setSpeed(out2);
      motor3.setSpeed(out3);
      
      pos_fb.data[0] = error[0];
      pos_fb.data[1] = error[1];
      pos_fb.data[2] = error[2];

      pos_pub.publish( &pos_fb );
      //nh.spinOnce();

      delay(10);
}

float computePID(int setpoint, int state, int channel,float _deadband){ //Written with global variable, make better by passing in pointers to abstract class
  //settled_flag[channel] = 0;

  currentTime[channel] = millis();
  elapsedTime[channel] = (currentTime[channel]-previousTime[channel])/1000;

  error[channel] = float(setpoint - state);

  if(abs(error[channel]) <= _deadband){
    error[channel] = 0;
    float out = 0;
    //settled_flag[channel] = 1;
    return out;
  }

  if(abs(cumError[channel]) >= (4*ROTATION)){ //Anti Windup
    cumError[channel] = 4*ROTATION;
  }

  cumError[channel] += error[channel]*elapsedTime[channel];
  rateError[channel] = (error[channel]-lastError[channel])/elapsedTime[channel];

  float out = kP[channel]*error[channel] + kI[channel]*cumError[channel] + kD[channel]*rateError[channel];

  lastError[channel] = error[channel];
  previousTime[channel] = currentTime[channel];

  return -out;
}


void limit_speed_percentage(float max_speed, float percent, int* output){
  if(*output > (max_speed*percent)){
    *output = max_speed*percent;
  }
  else if(*output < (-max_speed*percent)){
    *output = -max_speed*percent;
  }
}


void set_goal( const std_msgs::Int32MultiArray& goal_msg){

  setpoints[0] = goal_msg.data[0];
  setpoints[1] = goal_msg.data[1];
  setpoints[2] = goal_msg.data[2];
}