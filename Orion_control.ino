#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

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

// Variables used in ReadFromSerial function
char usersetting = 'e'; //e for speed control p for pwm control

//Motor stuff
motor Motor[] = {
  motor(10, 11, 12, 13), //RIGHT
  motor(14, 15, 26, 27),
  //                  motor(),
};

//IMU sensor related junk
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
sensors_event_t orientationData;
unsigned long starting = 0;
int delayTime = 0;

//IMU PID settings
float kp = 1.0;
float ki = 0;
float kd = 0;

//IMU PID variables
unsigned long currT = 0;
unsigned long prevT = 0;
double time_diff = 0;
float error = 0;
float dedt = 0;
float e_integral = 0.0;
float pre_error = 0.0;
float OriYtarget = -15.0;

//LEG
Servo leg[] = {
  Servo(),//left knee
  Servo(),//left thigh
  Servo(),//left thigh swivel
  Servo(),//right thigh swivel 
  Servo(),//right thigh
  Servo(),//right knee
  };



void setup(void)
{
  Wire.setSDA(0);
  Wire.setSCL(1);
  for (int i = 0; i<sizeof(leg)/sizeof(Servo);i++)
  {
    leg[i].attach(4+i);
    leg[i].write(90);
  }
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void loop(void)
{
  get_ori();
  starting = millis();
  //Start of time
  pos_control();



  //End of time
  delayTime = BNO055_SAMPLERATE_DELAY_MS-(millis()-starting);
  if (delayTime < 0)
  {
    int i = 0;
    while (1)
    {
      delayTime += i*BNO055_SAMPLERATE_DELAY_MS;
      i++;
      if (abs(delayTime)< BNO055_SAMPLERATE_DELAY_MS)
      {
        delayTime = BNO055_SAMPLERATE_DELAY_MS+delayTime;
      }
    }
  }
  delay(delayTime);
}

void get_ori()
{
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  Serial.print("ty= ");
  Serial.println(orientationData.orientation.y);
}

void askUserInput() {
  //This if statement just continuously asks for user input
  int num[] = {0, 0, 0, 0, 0, 0};
  if (Serial.available() > 0) { //keep asking for new user input without pausing
    String input = Serial.readString();
    //      Serial.println(input);
    char pre = usersetting;
    sscanf(input.c_str(), "%c %d %d %d %d %d %d", &usersetting, &num[0], &num[1], &num[2], &num[3], &num[4], &num[5]);
    //      Serial.println(num[0]);
    //      Serial.println(num[1]);
    switch (usersetting)
    {
      case 'e':
        for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
        {
          Motor[i].rpsInput = float(num[i]) / 100;
          Motor[i].e_integral = 0.0;
          Motor[i].pre_error = 0.0;
//          Motor[i].prevT = micros();
        }
        Motor[0].rpsInput *= -1;
        Serial.println("OK");
        break;
      case 'p':
        for (int i = 0; i < sizeof(Motor) / sizeof(motor); i++)
        {
          motor_control(Motor[i], num[i]);
          Motor[i].e_integral = 0.0;
          Motor[i].pre_error = 0.0;
          Motor[i].rpsInput = 0.0;
//          Motor[i].prevT = micros();
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

void motor_control(motor *Motor, int speed)
{
  if ((abs(speed)+abs(Motor->speedInput))!=abs(speed+Motor->speedInput))
  {
    if (speed < 0)
    {
      digitalWrite(Motor->PIN_BRAKE, HIGH);
      analogWrite(Motor->PIN_PWM , 0);
      delay(2);
      digitalWrite(Motor->PIN_DIR, 1);
    }
    else
    {
      digitalWrite(Motor->PIN_BRAKE, HIGH);
      analogWrite(Motor->PIN_PWM , 0);
      delay(2);
      digitalWrite(Motor->PIN_DIR, 0);
    }
  }
  Motor->speedInput = speed;
  digitalWrite(Motor->PIN_BRAKE, LOW);
  analogWrite(Motor->PIN_PWM , int(abs(Motor->speedInput)));
}

void pos_control()
{
  error = orientationData.orientation.y - OriYtarget;
  currT = micros();
  time_diff = double(currT-prevT)/(1.0e6);
  prevT = currT;
  dedt = (error - pre_error)/time_diff;
  e_integral += error*time_diff;
  pre_error = error;
  int u = kp*error + ki*e_integral + kd*dedt;
  u = constrain(u, -255, 255);
  Serial.print("C signal: ");
  Serial.println(u);
  
}
