/*************************************************** 
 *  This will drive 10 servos that each correspond to the 
 *  digit of pi. It will turn the servo and then turn it back
 *  in the order of pi
 ****************************************************/
 //add motors for wheels

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h> 
#include <SD.h>
#include "LowPower.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//350 - 180 degree
//SERVOMIN 50 is 45 degrees
// Every 50 is 22.5 degrees for servo max when servo min is 50
// Every 5 added to SERVOMAX adds 2.25 degrees
#define SERVOMIN  200// This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  300// This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Pin Declarations
const int8_t BackL = 2; //Back Left Bumper Pin
const int8_t BackR = 3; //Back Right bumper Pin
const int8_t FrontL = 4; //Front Left Bumper Pin
const int8_t FrontR = 5; //Front Right Bumper Pin
const int8_t MotorL = 6; //Left Motor Pin 
const int8_t MotorR = 7; //Right motor Pin
// Pin Declarations
uint8_t servonum;     //Which servos is being pushed
uint8_t Counter = 0;  //how many buttons should have been pushed
uint8_t FBumpLeft = 0, FBumpRight= 0, BBumpLeft = 0, BBumpRight = 0, startFlg = 0, BackBumpFlg = 0, FrontBumpFlg = 0; //DigitalRead for Digital Devices
uint8_t timerCount = 0, IFLG = 0;;
File myFile;

void temp(){}

void setup() {
  Serial.begin(9600);
  pinMode(FrontL, INPUT_PULLUP);
  pinMode(FrontR, INPUT_PULLUP);
  pinMode(BackL, INPUT_PULLUP);
  pinMode(BackR, INPUT_PULLUP);
  pinMode(MotorL, OUTPUT);
  pinMode(MotorR, OUTPUT);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  while(!SD.begin(10))
    Serial.println("initialization failed!");

  myFile = SD.open("pi.txt");
  
  //Read from SD Card
  servonum = myFile.read();
  switch(servonum)
  {
    case 48: servonum = 0; break;
    case 49: servonum = 1; break;
    case 50: servonum = 2; break;
    case 51: servonum = 3; break;
    case 52: servonum = 4; break;
    case 53: servonum = 5; break;
    case 54: servonum = 6; break;
    case 55: servonum = 7; break;
    case 56: servonum = 8; break;
    case 57: servonum = 9; break;
  }

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
  //Once the Nano is all set up puts it to sleep and waits for either Bumper to wake it up.
  //Then changes the flag to 1 so it won't go through this code again
  if (IFLG == 0)
  {
    attachInterrupt(digitalPinToInterrupt(BackR), temp, LOW);
    attachInterrupt(digitalPinToInterrupt(BackL), temp, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(BackR));
    detachInterrupt(digitalPinToInterrupt(BackL));
    IFLG = 1;
  }

  //Reads in the bumper value and creates flags for it to keep going when not pushed
  BBumpRight = digitalRead(BackR);
  BBumpLeft = digitalRead(BackL);
  FBumpRight = digitalRead(FrontR);
  FBumpLeft = digitalRead(FrontL);
  if(BBumpRight == LOW || BBumpLeft == LOW)
    BackBumpFlg = 1; 
  
  if(FBumpRight == LOW || FBumpLeft == LOW){
    FrontBumpFlg = 1;
    // Delay for Latch mecanism delay(200);
  }

  if(BackBumpFlg == 1)
  {
    //Turn Motor Controlls on  
  }
  
  if(FrontBumpFlg == 1)
  {
    BackBumpFlg = 0;
    //Turn Motor Controlls off
    
      if(startFlg == 0)
      {
        for (uint8_t i = 0; i < 15; i++)
        {
          for (uint16_t pulselen = 50; pulselen < 200; pulselen+=5)
          {
            pwm.setPWM(i, 0, pulselen);
          }
        }
        startFlg = 1;
      }
      
      //Pushing the servo forwards
      Serial.println(servonum);
      for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen+=5) {
        pwm.setPWM(servonum, 0, pulselen);
      }
      //Pulling the servo Back
      delay(500);
      for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen-=5) {
        pwm.setPWM(servonum, 0, pulselen);
      }

      delay(500);
      
      //Counter for Debugging
      Counter++;
      //Read in next byte in file
      servonum = myFile.read();
      //Filter the value
      switch(servonum)
      {
        case 48: servonum = 0; break;
        case 49: servonum = 1; break;
        case 50: servonum = 2; break;
        case 51: servonum = 3; break;
        case 52: servonum = 4; break;
        case 53: servonum = 5; break;
        case 54: servonum = 6; break;
        case 55: servonum = 7; break;
        case 56: servonum = 8; break;
        case 57: servonum = 9; break;
      }
  }
}

//Three minute timer
ISR(TIMER1_COMPA_vect){//timer1 interrupt
  {
    timerCount++;
    if(timerCount == 180)
    {
      Serial.println("3 Minutes");
      while(1){};
    }
  }
}

