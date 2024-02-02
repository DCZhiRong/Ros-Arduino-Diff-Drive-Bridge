/*   Hoverboard_Serial_Test
     Controls the speed, brake, and direction of a single hoverboard motor
     via commands sent through the serial port.
     Measures the speed of a hoverboard motor asynchronously
     using a custom ReadSpeed function.  Uses the SC speed pulse output of the
     RioRand 400W 6-60V PWM DC Brushless Electric Motor Speed Controller with Hall.
     Outputs the speed data to the serial port.

     created 2021
     Mad-EE  (Mad Electrical Engineer)
     www.mad-ee.com

     This example code is in the public domain.

     Platform:  Arduino UNO
*/
#include <ams_as5048b.h> //encoder library
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

// Constants
const double BAUD_RATE = 115200;                  // Serial port baud rate

// Variables used in ReadFromSerial function
char usersetting = 'e'; //e for speed control p for pwm control

//Pid stuff
//float kp = 1;
//float ki = 0;
//float kd = 0.00;
float kp = 2.0;
float ki = 25;
float kd = 0.005;

class motor {       // The class
  public:             // Access specifier
    // Pin Declarations
    int PIN_DIR;      // Motor direction signal
    int PIN_BRAKE;    // Motor brake signal (active low)
    int PIN_PWM;      // PWM motor speed control
    int PIN_EN;   // SC Speed Pulse Output from RioRand board
    unsigned long prevT = 0;
    float e_integral = 0.0;
    float pre_error = 0.0;
    double rpsMeasured = 0.0; //rps = rotations per second, rpsMeasured = rps of the wheel in real life
    double rpsInput = 0.0;
    double speedInput = 0.0;
    float prev_angle = 0;
    double pos = 0;
    unsigned long cur_angle_time, prev_angle_time;
    motor(int x, int y, int z, int a)
    {
      PIN_DIR = x;      // Motor direction signal
      PIN_BRAKE = y;    // Motor brake signal (active low)
      PIN_PWM = z;      // PWM motor speed control
      PIN_EN = a;
    }
};


motor Motor[] = {
  motor(6, 7, 10, 8), //RIGHT
  motor(2, 3, 11, 9),
  //                  motor(),
};

//motor & encoder setup
AMS_AS5048B mysensor[] = {
  AMS_AS5048B(0x40),//RIGHT
  AMS_AS5048B(0x42),
};


// This is ran only once at startup
void setup()
{
  // Set pin directions
  for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
  {
    pinMode(Motor[i].PIN_EN, OUTPUT);
    pinMode(Motor[i].PIN_PWM, OUTPUT);
    pinMode(Motor[i].PIN_BRAKE, OUTPUT);
    pinMode(Motor[i].PIN_DIR, OUTPUT);
    // Set initial pin states
    digitalWrite(Motor[i].PIN_BRAKE, false);
    digitalWrite(Motor[i].PIN_DIR, false);
    //digitalWrite(Motor[i].PIN_EN, false);
    analogWrite(Motor[i].PIN_PWM, 0);
    Motor[i].rpsInput = 0.0;
    Motor[i].prev_angle_time = micros();
    Motor[i].prevT = micros();
  }
  // Initialize serial port
  Serial.setTimeout(10);
  Serial.begin(BAUD_RATE);
  Wire.setSDA(0);
  Wire.setSCL(1);
  for (int i = 0; i < sizeof(mysensor) / sizeof(AMS_AS5048B); i ++)
  {
    mysensor[i].begin();
    mysensor[i].setZeroReg();
  }
  Serial.println("---- Program Started ----");
}

// This is the main program loop that runs repeatedly
void loop()
{
  askUserInput();
  switch (usersetting)
  {
    case 'e':
      for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
      {
        motor_control(&Motor[i]);
      }
      break;
    case 'p':
      for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
      {
        analogWrite(Motor[i].PIN_PWM , int((Motor[i].speedInput)));
        //Serial.println(int((Motor[i].speedInput)));
      }
      break;
  }
  // Read the speed from input pin (sets _rpm and _mph)
  for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
  {
    ReadSpeed(&Motor[i], &mysensor[i]);
  }
  //Serial.print("rpsMeasured: ");
  //Serial.println(Motor[1].rpsMeasured);
  // Serial.print(" | rpsInput: ");
  //Serial.println(Motor[0].speedInput);
}


void askUserInput() {
  //This if statement just continuously asks for user input
  int num[] = {0, 0};
  if (Serial.available() > 0) { //keep asking for new user input without pausing
    String input = Serial.readString();
    //      Serial.println(input);
    char pre = usersetting;
    sscanf(input.c_str(), "%c %d %d", &usersetting, &num[0], &num[1]);
    //      Serial.println(num[0]);
    //      Serial.println(num[1]);
    switch (usersetting)
    {
      case 'e':
        for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
        {
          Motor[i].rpsInput = float(num[i]) / 100;
          //              Motor[i].e_integral = 0.0;
          //              Motor[i].pre_error = 0.0;
          //              Motor[i].prevT = micros();
        }
        Motor[0].rpsInput *= -1;
        Serial.println("OK");
        break;
      case 'p':
        for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
        {
          Motor[i].speedInput = abs(num[i]);
          if (num[i] < 0)
          {
            digitalWrite(Motor[i].PIN_BRAKE, HIGH);
            analogWrite(Motor[i].PIN_PWM , 0);
            delayMicroseconds(1);
            digitalWrite(Motor[i].PIN_DIR, 1);
          }
          else
          {
            digitalWrite(Motor[i].PIN_BRAKE, HIGH);
            analogWrite(Motor[i].PIN_PWM , 0);
            delayMicroseconds(1);
            digitalWrite(Motor[i].PIN_DIR, 0);
          }
          digitalWrite(Motor[i].PIN_BRAKE, LOW);
          analogWrite(Motor[i].PIN_PWM, int((Motor[i].speedInput)));
          Motor[i].e_integral = 0.0;
          Motor[i].pre_error = 0.0;
          Motor[i].rpsInput = 0.0;
          Motor[i].prevT = micros();
        }
        Serial.println("OK");
        break;
      case 's':
        usersetting = pre;
        for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
        {
          Serial.print(float(Motor[i].rpsMeasured));
          Serial.print(" ");
        }
        Serial.println();
        Serial.println("OK");
        break;
      case 'a':
        usersetting = pre;
        for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
        {
          Serial.print(float(Motor[i].pos));
          Serial.print(" ");
        }
        Serial.println();
        Serial.println("OK");
        break;
    }
  }
}

// Reads the speed from the input pin and calculates RPM and MPH
// Monitors the state of the input pin and measures the time (µs) between pin transitions
void ReadSpeed(motor *Motor, AMS_AS5048B *mysensor)
{
  double cur_angle = (mysensor->angleR(U_DEG, true));
  long cur_angle_time = micros();
  double angle_diff = cur_angle - Motor->prev_angle;
  long angle_time_diff = cur_angle_time - Motor->prev_angle_time;
  if (angle_diff > 180)
  {
    angle_diff -= 2 * 180;
  }
  else if (angle_diff < -180)
  {
    angle_diff += 2 * 180;
  }
  Motor->pos = cur_angle * 100 * PI / 180;
  Motor->rpsMeasured = ((angle_diff * 1.0e6) / (angle_time_diff)) * PI / 180;
  Motor->prev_angle = cur_angle;
  Motor->prev_angle_time = cur_angle_time;
  //Serial.println(Motor->rpsMeasured);
}

void motor_control(motor *Motor) {
  float error = -Motor->rpsMeasured + Motor->rpsInput;
  unsigned long currT = micros();
  float time_diff = float((currT - (Motor->prevT))) / (1.0e6);
  Motor->prevT = currT;
  float dedt = (error - (Motor->pre_error)) / (time_diff);
  Motor->e_integral += (error) * time_diff;
  Motor->pre_error = error;
//    Serial.print("p ");
//    Serial.println(error);
//    Serial.print("i ");
//    Serial.println(Motor->e_integral);
//    Serial.print("d ");
//    Serial.println(dedt);
  float u = kp * error + kd * dedt + ki * (Motor->e_integral);
  u = constrain(u, -255, 255);
  if ((abs(u)+abs(Motor->speedInput))!=abs(u+Motor->speedInput))
  {
    if (u < 0)
    {
      digitalWrite(Motor->PIN_BRAKE, HIGH);
      analogWrite(Motor->PIN_PWM , 0);
      delay(1);
      digitalWrite(Motor->PIN_DIR, 1);
    }
    else
    {
      digitalWrite(Motor->PIN_BRAKE, HIGH);
      analogWrite(Motor->PIN_PWM , 0);
      delay(1);
      digitalWrite(Motor->PIN_DIR, 0);
    }
  }
  Motor->speedInput = u;
  digitalWrite(Motor->PIN_BRAKE, LOW);
  analogWrite(Motor->PIN_PWM , int(abs(Motor->speedInput)));
  //Serial.println(Motor->speedInput);
}
