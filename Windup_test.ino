#include <SD.h>
#include "LowPower.h"


// Pin Declarations
const int8_t BackL = 2; //Back Left Bumper Pin
const int8_t BackR = 3; //Back Right bumper Pin
const int8_t Motor2ENA = 6;
const int8_t MotorR1 = 17; //Right Motor Pin
const int8_t MotorR2 = 16; //Left Motor Pin

uint8_t BBumpLeft = 0, BBumpRight = 0, FrontBumpFlg = 0; //DigitalRead for Digital Devices

void temp(){}

void setup() {
  Serial.begin(9600);
  pinMode(BackL, INPUT_PULLUP);
  pinMode(BackR, INPUT_PULLUP);
  pinMode(MotorR2, OUTPUT);
  pinMode(MotorR1, OUTPUT);
  pinMode(Motor2ENA, OUTPUT);

    cli(); //stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //do interrupts
}

void loop() 
{ 

  //Reads in the bumper value and creates flags for it to keep going when not pushed
  BBumpRight = digitalRead(BackR);
  BBumpLeft = digitalRead(BackL);
  
  
  if(BBumpRight == LOW || BBumpLeft == LOW)
  {
    analogWrite(Motor2ENA, 250);
    Clockwise();
    delay(250);
    Stop();
    delay(5000);
    analogWrite(Motor2ENA, 150);
    CCW();
    delay(250);
    analogWrite(Motor2ENA, 100);
    delay(250);
    analogWrite(Motor2ENA, 50);
    delay(250);
    Stop();
  }
}

void Clockwise()
{
  digitalWrite(MotorR1, LOW);
  digitalWrite(MotorR2, HIGH);
}

void CCW ()
{
  digitalWrite(MotorR1, HIGH);
  digitalWrite(MotorR2, LOW);
}

void Stop()
{
  digitalWrite(MotorR1, LOW);
  digitalWrite(MotorR2, LOW);
}
