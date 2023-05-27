#include <Wire.h>
#include<ezButton.h>

#define stepPin 6
#define dirPin 7
#define LS_pin 8
#define Enable 5

#define blade1   4
//#define blade2   10
//#define bladepwm  5
int value = digitalRead(LS_pin);

ezButton limitSwitchHomeX(LS_pin);

int z = 0;
char level;
short steps;

void stepperHoming(void);
void levelAdjust(char level,short steps);
void setup()
{
  Serial.begin(9600);
  Wire.begin(0x12);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
   digitalWrite(Enable, LOW);
  limitSwitchHomeX.setDebounceTime(50);
  stepperHoming();
  // Start the I2C Bus as Slave on address 9


}
void receiveEvent(int bytes)
{
  z = Wire.read(); // read one character from the I2C
  if (z == 2) // level 1
  {
    levelAdjust(0, 200);
    delay(200);
  }
  else if  (z == 3)
  {
    levelAdjust(1, 200);
    delay(200);
  }

 else if (z == 4)
  {
bladeStart();
    delay(200);
  }
  else if (z == 5)
  {
    bladeStop();
    delay(200);
  }
  

void loop()
{

}

void stepperHoming(void)
{
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(Enable, OUTPUT);
  pinMode(LS_pin, INPUT);
  if (value == 1) // 1 aw 0 ala hasab el logic
  {
    digitalWrite(dirPin, LOW); // Enables the motor to move in a particular direction
    // Makes 200 pulses for making one full cycle rotation
   
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(700);    // by changing this time delay between the steps we can change the rotation speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(700);
    
      Serial.println(value);
  }
      Serial.print("value = ");
      Serial.println(value);

}

void levelAdjust(char level, short steps)
{

  switch (level)
  {
    case 0:
      Serial.println("level 0");
      break;
    case 1:
      Serial.println("level 1");
      digitalWrite(dirPin, HIGH);
      digitalWrite(Enable, LOW);
      for (int x = 0; x < steps; x++)
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(700);    // by changing this time delay between the steps we can change the rotation speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(700);
      }
      digitalWrite(Enable, HIGH);
      break;
  }
}

void bladeStart(void)
{
  Serial.println("cut");
  pinMode(blade1 , OUTPUT);
  //pinMode(blade2 , OUTPUT);
  //pinMode(bladepwm , OUTPUT);
  //analogWrite(bladepwm , 255);
  digitalWrite(blade1 , LOW);
  //digitalWrite(blade2 , LOW);
}

void bladeStop(void)
{
  Serial.println("stop cut");
  pinMode(blade1 , OUTPUT);
  // pinMode(blade2 , OUTPUT);
  //pinMode(bladepwm , OUTPUT);
  digitalWrite(blade1 , HIGH);
  //digitalWrite(blade2 , HIGH);
  delay(100);
  //digitalWrite(blade1 , LOW);
  //digitalWrite(blade2 , LOW);
  //delay(100);
}
