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
#define SERVOEVENMIN  280// This is the 'minimum' pulse length count (out of 4096) //even
#define SERVOEVENMAX  350// This is the 'maximum' pulse length count (out of 4096) //even
#define SERVOODDMIN   200
#define SERVOODDMAX   270
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Pin Declarations
const int8_t BackL = 2; //Back Left Bumper Pin
const int8_t BackR = 3; //Back Right bumper Pin
const int8_t Motor2ENA = 6;
const int8_t MotorR1 = 17; //Right Motor Pin
const int8_t MotorR2 = 16; //Left Motor Pin
 

int motor2Speed;
int motor1Speed;
int8_t timercountprev;

// Pin Declarations
uint8_t servonum;     //Which servos is being pushed
uint8_t Counter = 0;  //how many buttons should have been pushed
uint8_t BBumpLeft = 0, BBumpRight = 0, FrontBumpFlg = 0; //DigitalRead for Digital Devices
uint8_t timerCount = 0, IFLG = 0;;
File myFile;

void temp(){}

void setup() {
  Serial.begin(9600);
  pinMode(BackL, INPUT_PULLUP);
  pinMode(BackR, INPUT_PULLUP);
  pinMode(MotorR2, OUTPUT);
  pinMode(MotorR1, OUTPUT);
  pinMode(Motor2ENA, OUTPUT);
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  while(!SD.begin(10))
    Serial.println("initialization failed!");

  Serial.println("Init Success");
  delay(20);

  myFile = SD.open("pi.txt");
  
  //Read from SD Card
  servonum = myFile.read();
  Serial.println(servonum);
  SDtoServo(servonum);

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
  
  
  if(BBumpRight == LOW || BBumpLeft == LOW){
    FrontBumpFlg = 1;
    analogWrite(Motor2ENA, 250);
    digitalWrite(MotorR1, LOW);
    digitalWrite(MotorR2, HIGH);
    delay(1000);
    // Delay for Latch mecanism delay(200);
    digitalWrite(MotorR2  , LOW);
  }
  
  if(FrontBumpFlg == 1)
  {
      
      //Pushing the servo forwards
      Serial.println(servonum);
      switch(servonum)
      {
        case 0: runOdd(); break;
        case 1: runOdd(); break;
        case 2: runEven(); break;
        case 3: runOdd(); break;
        case 4: runEven(); break;
        case 5: runOdd(); break;
        case 6: runEven(); break;
        case 7: runOdd(); break;
        case 8: runEven(); break;
        case 9: runEven(); break;
      }
      
      //Counter for Debugging
      Counter++;
      //Read in next byte in file
      servonum = myFile.read();
      //Filter the value
      SDtoServo(servonum);
  }
}

//Three minute timer //timer1 interrupt
ISR(TIMER1_COMPA_vect)
{
   timerCount++;
  if(timerCount == 180)
  {
    Serial.println("3 Minutes");
    while(1){};
  }
}

void SDtoServo(uint8_t SDnum)
{
  switch(SDnum)
  {
    case 48: servonum = 9; break;
    case 49: servonum = 8; break;
    case 50: servonum = 7; break;
    case 51: servonum = 6; break;
    case 52: servonum = 5; break;
    case 53: servonum = 4; break;
    case 54: servonum = 3; break;
    case 55: servonum = 2; break;
    case 56: servonum = 1; break;
    case 57: servonum = 0; break;
   }
}

void runEven()
{
  for (uint16_t pulselen = SERVOODDMIN; pulselen < SERVOODDMAX; pulselen+=5) 
  pwm.setPWM(servonum, 0, pulselen);
  delay(250);
  for (uint16_t pulselen = SERVOODDMAX; pulselen > SERVOODDMIN; pulselen-=5) 
  pwm.setPWM(servonum, 0, pulselen);
  delay(250);
}

void runOdd()
{
  for (uint16_t pulselen = SERVOEVENMAX; pulselen > SERVOEVENMIN; pulselen-=5) 
  pwm.setPWM(servonum, 0, pulselen);
  delay(250); 
  for (uint16_t pulselen = SERVOEVENMIN; pulselen < SERVOEVENMAX; pulselen+=5)
  pwm.setPWM(servonum, 0, pulselen);
  delay(250);

}
