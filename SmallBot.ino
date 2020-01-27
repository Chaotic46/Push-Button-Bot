/*************************************************** 
 *  This will drive 10 servos that each correspond to the 
 *  digit of pi. It will turn the servo and then turn it back
 *  in the order of pi
 ****************************************************/

 //need to sleep the arduino until the bumper wakes it up. Disable uneeded timers and modules.
 //add front and back bump switches if back is 0 then run code

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SD.h>
#include <SPI.h> 
#include "LowPower.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
//350 - 180 degree
//SERVOMIN 50 is 45 degrees
// Every 50 is 22.5 degrees for servo max when servo min is 50
// Every 5 added to SERVOMAX adds 2.25 degrees
#define SERVOMIN  200// This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  300// This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum;
uint8_t Counter = 0;
uint8_t FrontBumper, BackBumper, BackFlg = 0, Front = 0, startFlg = 0;
const int wakeUpPin = 2;

void setup() {
  File myFile;
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  //pinMode(wakeUpPin, INPUT)
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  if (!SD.begin()){
    Serial.println("initialization failed!");
    while(1);
  }

  myFile = SD.open("pi.txt", FILE_READ);
  if (myFile){
    serial.println("pi.txt:");
  }
  //read from the file until there's nothing else in it:
  servonum = myFile.read();
  
}

void loop() 
{ 
  //Interrupt and sleep code:::
  //attachInterrupt( 0, pushPi, LOW)
  //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
      if(startFlg == 0)
      {
        for (uint8_t i = 0; i < 15; i++)
        {
          for (uint16_t pulselen = 50; pulselen < 200; pulselen+=5) {
          pwm.setPWM(i, 0, pulselen);
        }
      }
        startFlg = 1;
      }
      
      // Drive each servo one at a time using setPWM()
      Serial.println(servonum);
      for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen+=5) {
        pwm.setPWM(servonum, 0, pulselen);
      }

      delay(500);
      for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen-=5) {
        pwm.setPWM(servonum, 0, pulselen);
      }

      delay(500);
      Counter++;
      servonum = myFile.read();
}

//void pushPi()
//{
//  LowPower.idle(SLEEP_FOREVER, ADC_OFF, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_ON, USART0_OFF, TWI_ON);
//  while(1)
//  {
//        if(startFlg == 0)
//      {
//        for (uint8_t i = 0; i < 15; i++)
//        {
//          for (uint16_t pulselen = 50; pulselen < 200; pulselen+=5) {
//          pwm.setPWM(i, 0, pulselen);
//        }
//      }
//        startFlg = 1;
//      }
//      
//      // Drive each servo one at a time using setPWM()
//      Serial.println(servonum[Counter]);
//      for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen+=5) {
//        pwm.setPWM(servonum[Counter], 0, pulselen);
//      }
//
//      delay(500);
//      for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen-=5) {
//        pwm.setPWM(servonum[Counter], 0, pulselen);
//      }
//
//      delay(500);
//      Counter++;
//      if (Counter == 3)
//        Counter = 0;
//      servonum[Counter] = myFile.read();
//  }
//}

//Low Power example and Library
//https://github.com/rocketscream/Low-Power
//It looks like a new function is going to be created and theres is going ot be an interrupt pin on
//the back bumper to wake it up then it will stay awake.
