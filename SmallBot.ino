/*************************************************** 
 *  This will drive 10 servos that each correspond to the 
 *  digit of pi. It will turn the servo and then turn it back
 *  in the order of pi
 ****************************************************/

 //need to sleep the arduino until the bumper wakes it up. Disable uneeded timers and modules.
 //add front and back bump switches if back is 0 then run code

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
//350 - 180 degree
//SERVOMIN 50 is 45 degrees
// Every 50 is 22.5 degrees for servo max when servo min is 50
// Every 5 added to SERVOMAX adds 2.25 degrees
#define SERVOMIN  200// This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  300// This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum[100] = {3,1,4,1,5,9,2,6,5,3,5,8,9,7,9,3,2,3,8,4,6,2,6,4,3,3,8,3};
uint8_t Counter = 0;
uint8_t FrontBumper, BackBumper, BackFlg = 0, Front = 0, startFlg = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("8 channel Servo test!");

  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}

void loop() {

  if (BackBumper == 0)
  {
    BackFlg = 1;
  }
  
  //if (BackFlg == 1)
  //{
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
    Serial.println(servonum[Counter]);
    for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen+=5) {
      pwm.setPWM(servonum[Counter], 0, pulselen);
    }

    delay(500);
    for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen-=5) {
      pwm.setPWM(servonum[Counter], 0, pulselen);
    }

    delay(500);
    
    Counter++;
    if (Counter == 27)
      while(1);//spin loop
  //}
}
