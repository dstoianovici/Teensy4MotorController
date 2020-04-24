#include <Arduino.h>
//#include <QuadEncoder.h>
#include <Encoder.h>
#include <CytronMotorDriver.h>
#include <Serial_Parser.h>
//#include <std.h>


#define ENCODER_OPTIMIZE_INTERRUPTS

//Serial Comms
#define BAUD_RATE 115200
#define DELIM ','
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

#define LVL_SHIFT_EN 20

#define ENC1_A    0
#define ENC1_B    1

#define ENC2_A    2
#define ENC2_B    3

#define ENC3_A    4
#define ENC3_B    5

#define ENC4_A    8
#define ENC4_B    9

#define MOT1_PWM1 8
#define MOT1_PWM2 9
#define MOT1_EN   16

#define MOT2_PWM1 10
#define MOT2_PWM2 11
#define MOT2_EN   17

#define MOT3_PWM1 12
#define MOT3_PWM2 13
#define MOT3_EN   18

#define MOT4_PWM1 14
#define MOT4_PWM2 15
#define MOT4_EN   19

//Serial Parser Object Construct
Serial_Parser parser(DELIM,RANGE_MAX,RANGE_MIN);



Encoder enc1(ENC1_A,ENC1_B);
Encoder enc2(ENC2_A,ENC2_B);
Encoder enc3(ENC3_A,ENC3_B);
//Encoder enc3(ENC4_A,ENC4_B); //Usually Encoder 4

CytronMD motor1(PWM_DIR, MOT1_PWM1, MOT1_EN);
CytronMD motor2(PWM_DIR, MOT2_PWM1, MOT2_EN);
CytronMD motor3(PWM_DIR, MOT3_PWM1, MOT3_EN);
//CytronMD motor4(PWM_DIR, MOT4_PWM1, MOT4_EN);

//PID Function Prototype
float computePID(int setpoint, int state, int channel,float _deadband);
void limit_speed_percentage(float max_speed, float percent, int* output);

//Global Vals for PID control
int setpoints[NUM_PARAMS] = {MIDPOINT,MIDPOINT,MIDPOINT}, setpoints_old[NUM_PARAMS] = {MIDPOINT,MIDPOINT,MIDPOINT};

//PID Vars
// float kP[3] = {1.8,1.8,1.8}; //0.125
// float kI[3] = {0.1,0.1,0.1}; //0.001
// float kD[3] = {0.7,0.7,0.7};

float kP[3] = {1,1,1}; //0.125
float kI[3] = {0.0,0.0,0.0}; //0.001
float kD[3] = {0.1,0.1,0.1};

// float kD[4] = {0,0,0,0};
// float kI[4] = {0.0,0.0,0.0,0.0};

float deadband = 70.0;

int pars_params[2] = {0,0}; //[0] = num params [1] = in range (ignoring range for now)
int complete_flag = 0;

float volatile currentTime[NUM_MOTORS], previousTime[NUM_MOTORS], elapsedTime[NUM_MOTORS];
float volatile error[NUM_MOTORS]={0,0,0}, cumError[NUM_MOTORS]={0,0,0}, rateError[NUM_MOTORS]={0,0,0}, lastError[NUM_MOTORS]={0,0,0};

uint settled_flag[NUM_MOTORS] = {0,0,0};


void setup() {
  Serial.begin(115200);

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
  if(Serial.available() > 0){
    parser.GetParams(setpoints, pars_params);
    int pid_flag = 0;

    while(pid_flag == 0){
      if(settled_flag[0] == 1 && settled_flag[1] == 1 && settled_flag[2] == 1){
        pid_flag = 1;
        Serial.println("settled");
      }

      //Compute PID and send to motors.
      int out1 = computePID(setpoints[0], enc1.read(), 0, deadband);
      int out2 = computePID(setpoints[1], enc2.read(), 1, deadband);
      int out3 = computePID(setpoints[2], enc3.read(), 2, deadband);

      limit_speed_percentage(MAX_SPEED, SPEED_PRCNT, &out1);
      limit_speed_percentage(MAX_SPEED, SPEED_PRCNT, &out2);
      limit_speed_percentage(MAX_SPEED, SPEED_PRCNT, &out3);

      motor1.setSpeed(out1);
      motor2.setSpeed(out2);
      motor3.setSpeed(out3);

      delay(10);
    }
  }
}

float computePID(int setpoint, int state, int channel,float _deadband){ //Written with global variable, make better by passing in pointers to abstract class
  currentTime[channel] = millis();
  elapsedTime[channel] = (currentTime[channel]-previousTime[channel])/1000;

  error[channel] = float(setpoint - state);

  if(abs(cumError[channel]) >= (4*ROTATION)){ //Anti Windup
    cumError[channel] = 4*ROTATION;
  }

  if(abs(error[channel]) <= _deadband){
    error[channel] = 0;
    float out = 0;
    settled_flag[channel] = 1;
    return out;
  }

  else{
    settled_flag[channel] = 0;
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
