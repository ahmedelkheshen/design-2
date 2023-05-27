#include <Wire.h>


const int pingPin = 9;   // Trigger Pin of Ultrasonic Sensor
const int echoPin = 10;   // Echo Pin of Ultrasonic Sensor // left ultrasonic

const int pingPin_2 = 14;   // Trigger Pin of Ultrasonic Sensor
const int echoPin_2 = 15;  // right ultrasonic

const int pingPin_3 = 50;   // Trigger Pin of Ultrasonic Sensor
const int echoPin_3 = 51;  // back ultrasonic

int const IR_PIN = A0;              // analog pin for reading the IR sensor
int const NUM_READINGS_TO_AVG = 10; // Number of readings to average together
float const ADC_VOLTAGE = 5.0;      // Voltage that ADC is operating at.
float ADC_Step;

#define EN1_PIN   24
#define EN2_PIN   25
#define PWM_PIN   5 // left forward motor

#define EN1_PIN_M2   22
#define EN2_PIN_M2   23
#define PWM_PIN_M2   4 //  right forward motor

#define EN1_PIN_M3   26
#define EN2_PIN_M3   27
#define PWM_PIN_M3   8 //  left rear motor

#define EN1_PIN_M4   29
#define EN2_PIN_M4   28
#define PWM_PIN_M4   7 // right rear motor


char z;

long duration;
int cm;
char obstacle;

long duration_2;
int cm_2;




long duration_3;
int cm_3;

char state = 2; // lw 1 hshghl el project we lw zero hawa2af el project

void setup()
{
  Wire.begin(0x08);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);    // Starting Serial Terminal
  pinMode(EN1_PIN , OUTPUT);
  pinMode(EN2_PIN , OUTPUT);
  pinMode(PWM_PIN , OUTPUT);

  pinMode(EN1_PIN_M2 , OUTPUT);
  pinMode(EN2_PIN_M2 , OUTPUT);
  pinMode(PWM_PIN_M2 , OUTPUT);

  pinMode(EN1_PIN_M3 , OUTPUT);
  pinMode(EN2_PIN_M3 , OUTPUT);
  pinMode(PWM_PIN_M3 , OUTPUT);

  pinMode(EN1_PIN_M4 , OUTPUT);
  pinMode(EN2_PIN_M4 , OUTPUT);
  pinMode(PWM_PIN_M4 , OUTPUT);

  pinMode(pingPin, OUTPUT);
  pinMode(pingPin_2, OUTPUT);
  pinMode(pingPin_3, OUTPUT);

  pinMode(echoPin, INPUT);
  pinMode(echoPin_2, INPUT);
  pinMode(echoPin_3, INPUT);

  ADC_Step = ADC_VOLTAGE / 1024;
}

void receiveEvent(int bytes)
{
  z = Wire.read(); // read one character from the I2C
  if (z == 6) //
  {
    start_code(&state);
  }
  else //if (z == 7)
  {
    stop_code(&state);
  }
}


void loop()
{

  if (state == 1)
  {
    Serial.println("Start");
    ultrasonic1();
    ultrasonic2();

    if ((cm >= 50 && cm_2 >= 50) || cm == 0 || cm_2 == 0)
    {
      obstacle = 0;
    }
    else if ( cm_2 < 50 && cm > 50)
    {
      motor_stop();
      delay(2000);
    ultrasonic1();
    ultrasonic2();
      if (cm >= 50 && cm_2 >= 50) 
      {
        obstacle = 0;
      }
      else 
      {
        obstacle = 2;
      }
    }
    else if ( cm < 50 && cm_2 > 50)
    {
      motor_stop();
      delay(2000);
    ultrasonic1();
    ultrasonic2();
      if (cm >= 50 && cm_2 >= 50)
      {
        obstacle = 0;
      }
      else 
      {
        obstacle = 3;
      }
    }
    else
    {
      motor_stop();
      if (cm < 50 && cm_2 < 50)
      {
        obstacle = 1;
      }
      else 
      {
        obstacle = 0;
      }
    }

    motor_state(obstacle);
  }
  else if (state == 0)
  {
    Serial.println("el code wa2ef");
    motor_stop();
  }
}

void start_code(char * ptr)
{
  *ptr = 1;
}

void stop_code(char * ptr)
{
  *ptr = 0;
}


long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
void motor_state(char x)
{
  switch (x)
  {
    case 0:
      motor_forward();
      delay(1000);
      break;
    case 1:
      motor_stop();
      delay(1000);
      break;
    case 2:
      motor_left();
      delay(1000);
      break;
    case 3:
      motor_right();
      delay(1000);
      break;
  }
}

//////////////////////////////////////////////////////// EN1 LOW     EN2 HIGH 2 CW//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// EN1_M2 HIGH   EN2 LOW 2 CW/////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// EN1_M3 HIGH  EN2 LOW 2 CW/////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// EN1_M4 HIGH  EN2 LOW 2 CW/////////////////////////////////////////////////////////////////////

void motor_forward(void)
{
  analogWrite(PWM_PIN, 210); // 170  left forward
  analogWrite(PWM_PIN_M2, 175); //200 right forward
  analogWrite(PWM_PIN_M3, 210); //170 left rear
  analogWrite(PWM_PIN_M4, 175); //200 right rear
  Serial.println("move");

  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, HIGH);

  digitalWrite(EN1_PIN_M2, HIGH);
  digitalWrite(EN2_PIN_M2, LOW);

  digitalWrite(EN1_PIN_M3, HIGH);
  digitalWrite(EN2_PIN_M3, LOW);

  digitalWrite(EN1_PIN_M4, HIGH);
  digitalWrite(EN2_PIN_M4, LOW);
}
void motor_stop(void)
{
  Serial.println("stop");

  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, HIGH);

  digitalWrite(EN1_PIN_M2, HIGH);
  digitalWrite(EN2_PIN_M2, HIGH);

  digitalWrite(EN1_PIN_M3, HIGH);
  digitalWrite(EN2_PIN_M3, HIGH);

  digitalWrite(EN1_PIN_M4, HIGH);
  digitalWrite(EN2_PIN_M4, HIGH);
  delay(100);

  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, LOW);

  digitalWrite(EN1_PIN_M2, LOW);
  digitalWrite(EN2_PIN_M2, LOW);

  digitalWrite(EN1_PIN_M3, LOW);
  digitalWrite(EN2_PIN_M3, LOW);

  digitalWrite(EN1_PIN_M4, LOW);
  digitalWrite(EN2_PIN_M4, LOW);
}

void motor_left(void)
{
  Serial.println("na wa2ef we ha5osh ymeen ");
  Serial.println("na barga3");
  motorR_reverse();
  delay(500);
  analogWrite(PWM_PIN, 50);
  analogWrite(PWM_PIN_M2, 255);
  analogWrite(PWM_PIN_M3, 50);
  analogWrite(PWM_PIN_M4, 255);
  Serial.println("na bat7ark tany ymeen");

  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, HIGH);

  digitalWrite(EN1_PIN_M2, HIGH);
  digitalWrite(EN2_PIN_M2, LOW);

  digitalWrite(EN1_PIN_M3, HIGH);
  digitalWrite(EN2_PIN_M3, LOW);

  digitalWrite(EN1_PIN_M4, HIGH);
  digitalWrite(EN2_PIN_M4, LOW);
}

void motor_right(void)
{
  Serial.println("na wa2ef we ha5osh shemal ");
  Serial.println("na barga3");
  motorL_reverse();
  delay(500);
  analogWrite(PWM_PIN, 255);
  analogWrite(PWM_PIN_M2, 50);
  analogWrite(PWM_PIN_M3, 255);
  analogWrite(PWM_PIN_M4, 50);
  Serial.println("na bat7ark tany shemal");

  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, HIGH);

  digitalWrite(EN1_PIN_M2, HIGH);
  digitalWrite(EN2_PIN_M2, LOW);

  digitalWrite(EN1_PIN_M3, HIGH);
  digitalWrite(EN2_PIN_M3, LOW);

  digitalWrite(EN1_PIN_M4, HIGH);
  digitalWrite(EN2_PIN_M4, LOW);
}
void motorR_reverse(void) {
  analogWrite(PWM_PIN, 255); // left forward
  analogWrite(PWM_PIN_M2, 60); // right forward
  analogWrite(PWM_PIN_M3, 255); // left rear
  analogWrite(PWM_PIN_M4, 60); // right rear


  // if(cm_3 > 40)
  //{
  Serial.println(" right reverse");
  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, LOW);

  digitalWrite(EN1_PIN_M2, LOW);
  digitalWrite(EN2_PIN_M2, HIGH);

  digitalWrite(EN1_PIN_M3, LOW);
  digitalWrite(EN2_PIN_M3, HIGH);

  digitalWrite(EN1_PIN_M4, LOW);
  digitalWrite(EN2_PIN_M4, HIGH);
  delay(2500);
}

void motorL_reverse(void) {
  analogWrite(PWM_PIN, 60); // forward left
  analogWrite(PWM_PIN_M2, 240); // right forward
  analogWrite(PWM_PIN_M3, 60); // left rear
  analogWrite(PWM_PIN_M4, 240);// right rear

  //if(cm_3 > 40)
  //{
  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, LOW);

  digitalWrite(EN1_PIN_M2, LOW);
  digitalWrite(EN2_PIN_M2, HIGH);

  digitalWrite(EN1_PIN_M3, LOW);
  digitalWrite(EN2_PIN_M3, HIGH);

  digitalWrite(EN1_PIN_M4, LOW);
  digitalWrite(EN2_PIN_M4, HIGH);
  delay(2500);
}

void ultrasonic1(void)
{
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  // Measure the echo duration

  duration = pulseIn(echoPin, HIGH);

  // Convert duration to centimeters
  cm = duration * 0.034 / 2;//right ultrasonic
  Serial.print(" cm = ");
  Serial.println(cm);
}

void ultrasonic2(void)
{
  digitalWrite(pingPin_2, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin_2, LOW);

  // Measure the echo duration

  duration_2 = pulseIn(echoPin_2, HIGH);

  // Convert duration to centimeters
  cm_2 = duration_2 * 0.034 / 2; // left ultrasonic
  Serial.print(" cm 2 = ");
  Serial.println(cm_2);
}


void ultrasonic3(void)
{

  digitalWrite(pingPin_3, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_3, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin_3, LOW);

  // Measure the echo duration

  duration_3 = pulseIn(echoPin_3, HIGH);

  // Convert duration to centimeters
  cm_3 = duration_3 * 0.034 / 2;
}
