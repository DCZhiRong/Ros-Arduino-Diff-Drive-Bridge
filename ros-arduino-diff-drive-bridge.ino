#include <ams_as5048b.h> //encoder library
#include "CytronMotorDriver.h" //motor library

//encoder constants - copied from github, can just ignore
#define U_RAW 1
#define U_TRN 2
#define U_DEG 3
#define U_RAD 4
#define U_GRAD 5
#define U_MOA 6
#define U_SOA 7
#define U_MILNATO 8
#define U_MILSE 9
#define U_MILRU 10

//r2d2 constants
const float gearReduction = 0.5; //gear:wheel

//timer variables

//angles
float prev_angle1 = 0;
float prev_angle2 = 0;
float target_angle1 = 0.1;
float target_angle2 = 0.1;

//Speed control stuff
unsigned long cur_angle_time1, prev_angle_time1;
unsigned long cur_angle_time2, prev_angle_time2;
float rpsMeasured1 = 0.0; //rps = rotations per second, rpsMeasured = rps of the wheel in real life
float rpsInput1 = 0.0;
float rpsMeasured2 = 0.0; //rps = rotations per second, rpsMeasured = rps of the wheel in real life
float rpsInput2 = 0.0;
float speedInput1 = 0.0;
float speedInput2 = 0.0;

//Serial controls
#define inputString_buff 20
char usersetting = 'e'; //e for speed control p for pwm control
char inputString[inputString_buff];

//Pid stuff
unsigned long prevT1 = 0;
unsigned long prevT2 = 0;
float e_integral1 = 0.0;
float pre_error1 = 0.0;
float e_integral2 = 0.0;
float pre_error2 = 0.0;
float kp = 200;
float ki = 1300;
float kd = 0.001;
//motor & encoder setup
AMS_AS5048B mysensor[] = {
                            AMS_AS5048B(0x40),
                            AMS_AS5048B(0x40),
                         };

CytronMD motor[] = {
                    CytronMD(PWM_DIR, 3, 4), //motor1: AN1 = Pin 3, DIG1 = Pin 4
                    CytronMD(PWM_DIR, 5, 6), // motor2: An2 = Pin 5, DIG2 = Pin 6. 
                   };  

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
  for (int i = 0; i < sizeof(motor)/sizeof(CytronMD); i ++)
  {
    mysensor[i].begin(); 
    mysensor[i].setZeroReg();
  }
  prev_angle_time1 = micros();
  prev_angle_time2 = micros();
  prevT1 = micros();
  prevT2 = micros();
}

void loop() {
  askUserInput();
  switch(usersetting)
  {
    case 'e':
      //check_speed(&prev_angle1, &prev_angle_time1, &rpsMeasured1);
      motor_control(rpsMeasured1, rpsInput1, &speedInput1, &e_integral1, &pre_error1, &prevT1, motor[0]);
      motor_control(rpsMeasured2, rpsInput2, &speedInput2, &e_integral2, &pre_error2, &prevT2, motor[1] );
      break;
    case 'p':
      motor[0].setSpeed(speedInput1);
      motor[1].setSpeed(speedInput2);
      break;
  }
  //Serial.print("rpsMeasured: ");
  //Serial.print(rpsMeasured1);
  //Serial.print(" | rpsInput: ");
  //Serial.println(rpsInput1/2);
  check_speed(&prev_angle1, &prev_angle_time1, &rpsMeasured1, mysensor[0]);
  check_speed(&prev_angle2, &prev_angle_time2, &rpsMeasured2, mysensor[1]);

}

void askUserInput(){
  //This if statement just continuously asks for user input
  int num1 = 0;
  int num2 = 0;
  if (Serial.available()>0) { //keep asking for new user input without pausing
      String input = Serial.readString();
      char pre = usersetting;
      //input.toCharArray(inputString, inputString_buff);
      sscanf(input.c_str(), "%c %d %d", &usersetting, &num1, &num2);
      //Serial.println(num1);
      switch(usersetting)
      {
        case 'e':
          rpsInput1 = float(num1)/100;
          rpsInput2 = float(num2)/100;
          rpsInput1 = (rpsInput1)/(2*M_PI);
          rpsInput2 = (rpsInput2)/(2*M_PI);
          e_integral1 = 0.0;
          pre_error1 = 0.0;
          e_integral2 = 0.0;
          pre_error2 = 0.0;
          prevT1 = micros();
          prevT2 = micros();
          Serial.println("OK");
          break;
        case 'p':
          speedInput1 = num1;
          speedInput2 = num2;
          Serial.println("OK");
          break;
        case 's':
          usersetting = pre;
          Serial.print(rpsMeasured1*2*M_PI);
          Serial.print(" ");
          Serial.println(rpsMeasured2*2*M_PI);
          break;
        case 'a':
          usersetting = pre;
          Serial.print(mysensor[0].angleR(U_RAD, true));
          Serial.print(" ");
          Serial.println(mysensor[1].angleR(U_RAD, true));
          break;
      }
    }
  }


void motor_control(float rpsMeasured, float rpsInput, float *speedInput, float *e_integral, float *pre_error, unsigned long *prevT, CytronMD motor){
  float error = -rpsMeasured+rpsInput;
  unsigned long currT = micros();
  float time_diff = float((currT - (*prevT)))/(1.0e6);
  *prevT = currT;
  float dedt = (error-(*pre_error))/(time_diff);
  *e_integral += error*time_diff;
  float u = kp*error + kd*dedt + ki*(*e_integral);
  *speedInput = u;
  *speedInput = constrain(*speedInput, -255, 255);
  //Serial.println(*speedInput);
  motor.setSpeed(*speedInput);
}

void check_speed(float *prev_angle, unsigned long *prev_angle_time, float *rpsMeasured, AMS_AS5048B mysensor)
{
  float cur_angle=(mysensor.angleR(U_DEG, true));
  long cur_angle_time = micros();
  float angle_diff = cur_angle-*prev_angle;
  long angle_time_diff = cur_angle_time-*prev_angle_time;
  if (angle_diff > 180)
  {
    angle_diff -= 360;
  }
  else if (angle_diff < -180)
  {
    angle_diff += 360;
  }
  *rpsMeasured=(angle_diff*1.0e6)/(angle_time_diff*360);
  *prev_angle = cur_angle;
  *prev_angle_time = cur_angle_time;
}
