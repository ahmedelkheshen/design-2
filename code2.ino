/*
   The purpose of this code is to count the ouput pulses or
   the encoder outputs as you rotate the Motor shaft. You can run the
   same code on the Arduino Uno, Arduino Nano, Arduino Mega, etc.
*/
#include <Wire.h>

/******************Position Control*************************/

#define Encoder_output_A 2 // pin2 of the Arduino
#define Encoder_output_B 18 // pin 3 of the Arduino

#define Encoder2_output_A 3 // pin2 of the Arduino
#define Encoder2_output_B 19 // pin 3 of the Arduino  

const int pingPin = 9;   // Trigger Pin of Ultrasonic Sensor
const int echoPin = 10;   // Echo Pin of Ultrasonic Sensor // left ultrasonic

const int pingPin_2 = 14;   // Trigger Pin of Ultrasonic Sensor
const int echoPin_2 = 15;  // right ultrasonic

const int pingPin_3 = 11;   // Trigger Pin of Ultrasonic Sensor
const int echoPin_3 = 12;  // back ultrasonic


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
// these two pins has the hardware interrupts as well.

int const IR_PIN = A0;              // analog pin for reading the IR sensor
int const NUM_READINGS_TO_AVG = 10; // Number of readings to average together
float const ADC_VOLTAGE = 5.0;      // Voltage that ADC is operating at.
float ADC_Step;

#define SIZE 6

#define pulse_per_rev  (386.0)
#define perimetre (0.4082)

int len[SIZE]; //lenght in m

float Count_pulses = 0.0;
int rev_target[SIZE];//length / perimetre
float rev_num = Count_pulses / pulse_per_rev;

float Count_pulses2 = 0;
float rev_num2 = Count_pulses2 / pulse_per_rev;

char z;

char cam =0;

int real_rev = Count_pulses / pulse_per_rev;;
int real_rev2 = Count_pulses2 / pulse_per_rev;
char counter;

long duration, cm;
long duration_2, cm_2;
long duration_3, cm_3;
float sharp_cm;


char state = 2; 

/**************Functions Prototypes*******************/
void receiveEvent(int bytes);
void readEncoder_M2();
void readEncoder();
void mapDimensions(int *ptr);
void motorMovement(void);
void INIT(void);
void motorStop(void);
void revolutionCalculate(int *ptr1);
void motorFirstLane(int *ptr);
void motorRightReverse(void);
void motorLeftReverse(void);
void motorSecondLane(int *ptr);
void motorMovementReverse(void);
void motorMoveRight(void);
void motorMoveLeft(void);
void motorThirdLane(int *ptr);
void motorFourthLane(int *ptr);
void motorFifthLane(int *ptr);
void motorSixthLane(int *ptr);
void start_code(char * ptr);
void stop_code(char * ptr);
void motor_stop(void);
/********************************************************/

void setup()
{
  Serial.begin(9600); // activates the serial communication
  Wire.begin(0x08);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  INIT();

  mapDimensions(len);
  delay(1000);
  revolutionCalculate(rev_target);
  real_rev = 0;
  real_rev2 = 0;
}
/********************************************************/
void receiveEvent(int bytes)
{
  z = Wire.read(); // read one character from the I2C
  if (z == 1) //
  {
    cam = 1;
  }
  else if (z == 0)
  {
    cam = 0;
  }
 else if (z == 6) //
  {
    start_code(&state);
  }
  else if (z == 7)
  {
    stop_code(&state);
  }
}
/**********************************************************/
void loop()
{
  /* Serial.println("Pulses: ");
    Serial.println(Count_pulses);*/

  if (state == 1)
  {
  for (counter = 0; counter < SIZE; counter++)
  {
    switch (counter)
    {
      case 0:
        //real_rev = 100;//for testing
        motorFirstLane(rev_target);
        break;
      case 1:
          motorRightReverse();
        break;
      case 2:
        motorMoveRight();
        delay(500);
        motorMoveLeft();
        delay(500);
        motorThirdLane(rev_target);
        break;
      case 3:
        motorRightReverse();
        delay(500);
        motorLeftReverse();
        delay(500);
        motorFourthLane(rev_target);
        break;
      case 4:
        motorMoveRight();
        delay(500);
        motorMoveLeft();
        delay(500);
        motorFifthLane(rev_target);
        break;
      case 5:
        motorRightReverse();
        delay(500);
        motorLeftReverse();
        delay(500);
        motorSixthLane(rev_target);
        break;
    
    
      }
    }
  while (1);
  }
  else if (state == 0)
  {
    Serial.println("el code wa2ef");
    motor_stop();
  }
 }
  
/**********************************************************************/
void readEncoder()
{
  int b = digitalRead(Encoder_output_B);
  if (b > 0)
  {
    Count_pulses = Count_pulses + 1;
  }
  else
  {
    Count_pulses = Count_pulses - 1;
  }
}

void readEncoder_M2()
{
  int b = digitalRead(Encoder2_output_B);
  if (b > 0)
  {
    Count_pulses2 = Count_pulses2 + 1;
  }
  else
  {
    Count_pulses2 = Count_pulses2 - 1;
  }
}

void mapDimensions(int *ptr)
{
  int x;
  Serial.println("gowa fn loop");
  for (x = 0; x < SIZE; x++)
  {
    while (cam != 0)
    {
      Serial.println("gowa while loop");
      motorMovement();
      //delay(1000);//testing
    }
    if (cam == 0)
    {
      motorStop();
      ptr[x] = rev_num * perimetre ;
      rev_num = 0;
      Serial.println("gowa forrrr loop");
    }
    Serial.println("kharag mn if");
    Serial.println(x);
  }
}

void motorMovement(void)
{
  /*int y;
    for(
    while (rev_num <= rev_target)
    {
    analogWrite(pwmPin, 255);
    digitalWrite(MotorPin1, HIGH);
    digitalWrite(MotorPin2, LOW);
    Serial.println("rev num: ");
    Serial.println(rev_num);
    }
    delay(1000);
    Serial.plot(Count_pulses);*/
  Serial.println("move");
  analogWrite(PWM_PIN, 200); //168 rpm = 200  left forward
  analogWrite(PWM_PIN_M2, 230); //230 right forward
  analogWrite(PWM_PIN_M3, 200); //200 left rear
  analogWrite(PWM_PIN_M4, 230); //230 right rear
  // Serial.println("move");

  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, HIGH);

  digitalWrite(EN1_PIN_M2, HIGH);
  digitalWrite(EN2_PIN_M2, LOW);

  digitalWrite(EN1_PIN_M3, HIGH);
  digitalWrite(EN2_PIN_M3, LOW);

  digitalWrite(EN1_PIN_M4, HIGH);
  digitalWrite(EN2_PIN_M4, LOW);
  delay(500);
}

void INIT(void)
{
  pinMode(Encoder_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_B, INPUT); // sets the Encoder_output_B pin as the input

  pinMode(Encoder2_output_A, INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder2_output_B, INPUT); // sets the Encoder_output_B pin as the input

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


  attachInterrupt(digitalPinToInterrupt(Encoder_output_A), readEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder2_output_A), readEncoder_M2, RISING);

}

void motorStop(void)
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
  delay(2000);
}

void revolutionCalculate(int *ptr1)
{
  for (int i = 0; i < SIZE; i++)
  {
    ptr1[i] = len[i] / perimetre;
    Serial.println("byhfaz");
    Serial.println(i);
    delay(500);
  }
}

void motorFirstLane(int *ptr)
{

  while (real_rev <= ptr[counter])
  {
    motorMovement();
    Serial.println(" bymshy lhd el akher odam fy awl lane ");
    delay(1500);
    // real_rev = 200;
  }
  Serial.println("we2ef fy akher el map");
  real_rev = 0;
  motorStop();

}

void motorRightReverse(void)
{
  analogWrite(PWM_PIN, 255); // left forward
  analogWrite(PWM_PIN_M2, 50); // right forward
  analogWrite(PWM_PIN_M3, 255); // left rear
  analogWrite(PWM_PIN_M4, 50); // right rear

  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, LOW);

  digitalWrite(EN1_PIN_M2, LOW);
  digitalWrite(EN2_PIN_M2, HIGH);

  digitalWrite(EN1_PIN_M3, LOW);
  digitalWrite(EN2_PIN_M3, HIGH);

  digitalWrite(EN1_PIN_M4, LOW);
  digitalWrite(EN2_PIN_M4, HIGH);
  delay(1000);
  real_rev = 0;
  Serial.println("hwa rage3 byksr ymeen");
  motorStop();
}

void motorLeftReverse(void)
{
  analogWrite(PWM_PIN, 50); // forward left
  analogWrite(PWM_PIN_M2, 255); // right forward
  analogWrite(PWM_PIN_M3, 50); // left rear
  analogWrite(PWM_PIN_M4, 255);// right rear

  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, LOW);

  digitalWrite(EN1_PIN_M2, LOW);
  digitalWrite(EN2_PIN_M2, HIGH);

  digitalWrite(EN1_PIN_M3, LOW);
  digitalWrite(EN2_PIN_M3, HIGH);

  digitalWrite(EN1_PIN_M4, LOW);
  digitalWrite(EN2_PIN_M4, HIGH);
  delay(500);
  real_rev = 0;
  Serial.println("hwa rage3 byksr shemal");
  motorStop();
}

void motorSecondLane(int *ptr)
{
  while (real_rev <= ptr[counter]) // na2esha testing bs ashoof we hwa rage3 byakhod kam revolution
  {
    motorMovementReverse();
    Serial.println("bymshy lhd el akher wara fy tani lane");
    delay(1500);
    // real_rev = 200;
  }
  Serial.println("we2ef fy akher el map");
  real_rev = 0;
  motorStop();
}

void motorMovementReverse(void)
{
  analogWrite(PWM_PIN, 200); // left forward
  analogWrite(PWM_PIN_M2, 230); // right forward
  analogWrite(PWM_PIN_M3, 200); // left rear
  analogWrite(PWM_PIN_M4, 230); // right rear

  digitalWrite(EN1_PIN, HIGH);
  digitalWrite(EN2_PIN, LOW);

  digitalWrite(EN1_PIN_M2, LOW);
  digitalWrite(EN2_PIN_M2, HIGH);

  digitalWrite(EN1_PIN_M3, LOW);
  digitalWrite(EN2_PIN_M3, HIGH);

  digitalWrite(EN1_PIN_M4, LOW);
  digitalWrite(EN2_PIN_M4, HIGH);
  Serial.println("hwa rage3 straight forward");
}

void motorMoveRight(void)
{
  analogWrite(PWM_PIN, 255);
  analogWrite(PWM_PIN_M2, 50);
  analogWrite(PWM_PIN_M3, 255);
  analogWrite(PWM_PIN_M4, 50);

  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, HIGH);

  digitalWrite(EN1_PIN_M2, HIGH);
  digitalWrite(EN2_PIN_M2, LOW);

  digitalWrite(EN1_PIN_M3, HIGH);
  digitalWrite(EN2_PIN_M3, LOW);

  digitalWrite(EN1_PIN_M4, HIGH);
  digitalWrite(EN2_PIN_M4, LOW);
  delay(1000);
  Serial.println("na balef ymeen");
  real_rev = 0;
  motorStop();
}

void motorMoveLeft(void)
{
  analogWrite(PWM_PIN, 50);
  analogWrite(PWM_PIN_M2, 255);
  analogWrite(PWM_PIN_M3, 50);
  analogWrite(PWM_PIN_M4, 255);


  digitalWrite(EN1_PIN, LOW);
  digitalWrite(EN2_PIN, HIGH);

  digitalWrite(EN1_PIN_M2, HIGH);
  digitalWrite(EN2_PIN_M2, LOW);

  digitalWrite(EN1_PIN_M3, HIGH);
  digitalWrite(EN2_PIN_M3, LOW);

  digitalWrite(EN1_PIN_M4, HIGH);
  digitalWrite(EN2_PIN_M4, LOW);
  delay(500);
  Serial.println("na balef shemal");
  real_rev = 0;
  motorStop();
}

void motorThirdLane(int *ptr)
{
  while (real_rev <= ptr[counter]) // na2esha testing bs ashoof we hwa rage3 byakhod kam revolution
  {
    motorMovement();
    Serial.println("bymshy lhd el akher odam fy talet lane");
    delay(1500);
    // real_rev = 200;
  }
  Serial.println("we2ef fy akher el map");
  real_rev = 0;
  motorStop();
}

void motorFourthLane(int *ptr)
{
  while (real_rev <= ptr[counter]) // na2esha testing bs ashoof we hwa rage3 byakhod kam revolution
  {
    motorMovementReverse();
    Serial.println("bymshy lhd el akher wara fy rabe3 lane");
    delay(1500);
    // real_rev = 200;
  }
  Serial.println("we2ef fy akher el map");
  real_rev = 0;
  motorStop();
}

void motorFifthLane(int *ptr)
{
  while (real_rev <= ptr[counter]) // na2esha testing bs ashoof we hwa rage3 byakhod kam revolution
  {
    motorMovement();
    Serial.println("bymshy lhd el akher odam fy khames lane");
    delay(1500);
    // real_rev = 200;
  }
  Serial.println("we2ef fy akher el map");
  real_rev = 0;
  motorStop();
}

void motorSixthLane(int *ptr)
{
  while (real_rev <= ptr[counter]) // na2esha testing bs ashoof we hwa rage3 byakhod kam revolution
  {
    motorMovementReverse();
    Serial.println("bymshy lhd el akher wara fy sades lane");
    delay(1500);
    // real_rev = 200;
  }
  Serial.println("we2ef fy akher el map");
  real_rev = 0;
  motorStop();
}

void ultrasonic3(void)
{
  pinMode(pingPin_3, OUTPUT);
  digitalWrite(pingPin_3, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_3, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin_3, LOW);

  // Measure the echo duration
  pinMode(echoPin_3, INPUT);
  duration_3 = pulseIn(echoPin_3, HIGH);

  // Convert duration to centimeters
  cm_3 = microsecondsToCentimeters(duration_3);//right ultrasonic
}

float sharpIR(void)
{
  float sum = 0;
  for (int i = 0; i < NUM_READINGS_TO_AVG; i++) // Take the number of readings that you want to average
  {
    sum += analogRead(IR_PIN);
  }
  float volts = sum / NUM_READINGS_TO_AVG * ADC_Step;  // Calculate average * volts per ADC step
  float distance = 65 * pow(volts, -1.10);  // Calculate distance from voltage.  Formula attributed to luckylarry.co.uk
  Serial.print(distance);                   // Printout the results to the Serial Monitor
  Serial.println(" cm ");
  /*Serial.print(distance/2.54);
    Serial.println(" in");
  */
  delay(500);
  return distance;
}

void ultrasonic2(void)
{
  pinMode(pingPin_2, OUTPUT);
  digitalWrite(pingPin_2, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin_2, LOW);

  // Measure the echo duration
  pinMode(echoPin_2, INPUT);
  duration_2 = pulseIn(echoPin_2, HIGH);

  // Convert duration to centimeters
  cm_2 = microsecondsToCentimeters(duration_2); // left ultrasonic
}

void ultrasonic1(void)
{
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);

  // Measure the echo duration
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // Convert duration to centimeters
  cm = microsecondsToCentimeters(duration);//right ultrasonic
}

void motor_right(void)
{
  delay(500);
  motorStop();
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
  delay(1000);

}

void motorR_reverse(void) {
  analogWrite(PWM_PIN, 255); // left forward
  analogWrite(PWM_PIN_M2, 50); // right forward
  analogWrite(PWM_PIN_M3, 255); // left rear
  analogWrite(PWM_PIN_M4, 50); // right rear


  if (cm_3 > 40)
  {
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
  else
  {
    motorStop();
  }
}

void motorL_reverse(void) {
  analogWrite(PWM_PIN, 50); // forward left
  analogWrite(PWM_PIN_M2, 255); // right forward
  analogWrite(PWM_PIN_M3, 50); // left rear
  analogWrite(PWM_PIN_M4, 255);// right rear

  if (cm_3 > 40)
  {
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
  else
  {
    motorStop();
  }
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void start_code(char * ptr)
{
  *ptr = 1;
}

void stop_code(char * ptr)
{
  *ptr = 0;
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
