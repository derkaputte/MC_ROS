#include <Arduino.h>
#include "RunningMedian.h"
#include "TimerThree.h"
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;

#define pinMotorEnable  37         // EN motors enable
#define pinMotorLeftPWM 5           // M1_IN1 left motor PWM pin
#define pinMotorLeftDir 31          // M1_IN2 left motor Dir pin
#define pinMotorLeftSense A1       // M1_FB  left motor current sense
#define pinMotorLeftFault 25       // M1_SF  left motor fault

#define pinMotorRightPWM 3           // M2_IN1 left motor PWM pin
#define pinMotorRightDir 33          // M2_IN2 left motor Dir pin
#define pinMotorRightSense A0       // M2_FB  left motor current sense
#define pinMotorRightFault 27       // M2_SF  left motor fault


#define pinOdometryLeft A12      // left odometry sensor
#define pinOdometryLeft2 A13     // left odometry sensor (optional two-wire)
#define pinOdometryRight A14     // right odometry sensor 
#define pinOdometryRight2 A15    // right odometry sensor (optional two-wire)  

bool twoWayOdometrySensorUse = 1;
bool odometryRightSwapDir = 0;       // inverse right encoder direction?
bool odometryLeftSwapDir = 0;       // inverse left encoder direction?        

int odometryLeft ;   // left wheel counter
int odometryRight ;  // right wheel counter

int motorLeftPWMCurr = 255;
int motorRightPWMCurr = 255;

boolean odometryLeftLastState;
boolean odometryLeftLastState2;
boolean odometryRightLastState;
boolean odometryRightLastState2;
    


void motor_speed_l_cb( const std_msgs::UInt16& cmd_msg, int speed){
  speed = cmd_msg.data; //set servo angle, should be from 0-180  
}

ros::Subscriber<std_msgs::UInt16> sub("servo", motor_speed_l_cb);

// Determines the rotation count and direction of the odometry encoders. Called in the odometry pins interrupt.
// encoder signal/Ardumower pinout etc. at http://wiki.ardumower.de/index.php?title=Odometry
// Logic is: 
//    If the pin1 change transition (odometryLeftState) is LOW -> HIGH... 
//      If the pin2 current state is HIGH :  step count forward   (odometryLeft++)
//        Otherwise :  step count reverse   (odometryLeft--)   
// odometryState:  1st left and right odometry signal
// odometryState2: 2nd left and right odometry signal (optional two-wire encoders)
void setOdometryState(unsigned long timeMicros, boolean odometryLeftState, boolean odometryRightState, boolean odometryLeftState2, boolean odometryRightState2){
  int leftStep = 1;
  int rightStep = 1;
  if (odometryLeftSwapDir) leftStep = -1;
  if (odometryRightSwapDir) rightStep = -1;
  if (odometryLeftState != odometryLeftLastState){    
    if (odometryLeftState){ // pin1 makes LOW->HIGH transition
      if (twoWayOdometrySensorUse) { 
        // pin2 = HIGH? => forward 
        if (odometryLeftState2) odometryLeft += leftStep; else odometryLeft -= leftStep;
      } 
      else { 
         if (motorLeftPWMCurr >=0) odometryLeft ++; else odometryLeft --;
      }
    }
    odometryLeftLastState = odometryLeftState;
  } 

  if (odometryRightState != odometryRightLastState){
    if (odometryRightState){ // pin1 makes LOW->HIGH transition
      if (twoWayOdometrySensorUse) {
        // pin2 = HIGH? => forward
        if (odometryRightState2) odometryRight += rightStep; else odometryRight -= rightStep;
      }     
      else {
         if (motorRightPWMCurr >=0) odometryRight ++; else odometryRight --;    
      }
    }
    odometryRightLastState = odometryRightState;
  }  
  if (twoWayOdometrySensorUse) {
    if (odometryRightState2 != odometryRightLastState2){
      odometryRightLastState2 = odometryRightState2;    
    }  
    if (odometryLeftState2 != odometryLeftLastState2){
      odometryLeftLastState2 = odometryLeftState2;    
    }
  }    
}



// odometry signal change interrupt
ISR(PCINT2_vect, ISR_NOBLOCK){
  unsigned long timeMicros = micros();
  boolean odometryLeftState = digitalRead(pinOdometryLeft);
  boolean odometryLeftState2 = digitalRead(pinOdometryLeft2);
  boolean odometryRightState = digitalRead(pinOdometryRight);  
  boolean odometryRightState2 = digitalRead(pinOdometryRight2);  
  setOdometryState(timeMicros, odometryLeftState, odometryRightState, odometryLeftState2, odometryRightState2);   
}




//#define pinOdometryLeft A12      // left odometry sensor


// RunningMedian<int,50> measurements;
// unsigned long ticks = 0;


//ISR(PCINT2_vect){
//  if (digitalRead(pinOdometryLeft)) ticks++;
//}



// MC33926 motor driver
// Check http://forum.pololu.com/viewtopic.php?f=15&t=5272#p25031 for explanations.
//(8-bit PWM=255, 10-bit PWM=1023)
// IN1 PinPWM         IN2 PinDir
// PWM                L     Forward
// nPWM               H     Reverse

void setMC33926(int pinDir, int pinPWM, int speed) {
  speed *= 4;
  if (speed < 0) {
    digitalWrite(pinDir, HIGH ) ;
    //analogWrite(pinPWM, 255 - ((byte)abs(speed)));
    Timer3.setPwmDuty(pinPWM, 1023 - abs(speed));
  } else {
    digitalWrite(pinDir, LOW) ;
    //analogWrite(pinPWM, ((byte)speed));
    Timer3.setPwmDuty(pinPWM, speed);
  }
}


void setup()
{
  Serial.begin(19200);
   
  // motor enable
  pinMode(pinMotorEnable, OUTPUT);
  digitalWrite(pinMotorEnable, HIGH);
  
  // left wheel motor
  pinMode(pinMotorLeftPWM, OUTPUT);  
  pinMode(pinMotorLeftDir, OUTPUT);
  pinMode(pinMotorLeftSense, INPUT);
  pinMode(pinMotorLeftFault, INPUT);
  
  // right wheel motor
  pinMode(pinMotorRightPWM, OUTPUT);  
  pinMode(pinMotorRightDir, OUTPUT);
  pinMode(pinMotorRightSense, INPUT);
  pinMode(pinMotorRightFault, INPUT);

  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
 
  nh.initNode();
  nh.subscribe(sub);

  Serial.println("START");  
  
  // odometry
  pinMode(pinOdometryLeft, INPUT_PULLUP);  
  pinMode(pinOdometryLeft2, INPUT_PULLUP);    
  pinMode(pinOdometryRight, INPUT_PULLUP);
  pinMode(pinOdometryRight2, INPUT_PULLUP);    
  
  
  // enable odometry interrupts
  PCICR |= (1<<PCIE2);
  PCMSK2 |= (1<<PCINT20);
  PCMSK2 |= (1<<PCINT21);  
  PCMSK2 |= (1<<PCINT22);
  PCMSK2 |= (1<<PCINT23);                


  
//  pinMode(pinOdometryLeft, INPUT_PULLUP);
//  // http://wiki.ardumower.de/images/a/ad/Ardunio_mega_pinout.png
//  PCICR |= (1<<PCIE2);
//  PCMSK2 |= (1<<PCINT20);
//  
//  // http://sobisource.com/arduino-mega-pwm-pin-and-frequency-timer-control/
//  // http://www.atmel.com/images/doc2549.pdf
//  //TCCR3B = (TCCR3B & 0xF8) | 0x02;    // set PWM frequency 3.9 Khz (pin2,3,5)   
  
  Timer3.initialize(50);
  Timer3.pwm(pinMotorLeftPWM, 511);                  
  Timer3.pwm(pinMotorRightPWM, 511);                  
  
  Serial.println("SETUP");
}


void loop()
{
  Serial.println("pwm_l,pwm_r,sense");
  nh.spinOnce();
  delay(1);
}

void beschleunigung() {
    for (int speed=0; speed <= 125; speed += 10){    
    setMC33926(pinMotorLeftDir, pinMotorLeftPWM, speed);
    setMC33926(pinMotorRightDir, pinMotorRightPWM, speed);
    delay(5);
    for (int i=0; i < 10; i++){
      int sense = 0;
      sense = analogRead(pinMotorLeftSense);
      Serial.print(speed);
      Serial.print(",");    
      Serial.print(sense);
      Serial.print(",");    
      Serial.print(odometryLeft);    
      Serial.print(",");    
      Serial.print(odometryRight);    
      Serial.println();       
    }    
  } 
}

void bremsen() {
    for (int speed=125; speed >= 0; speed -= 10){    
    setMC33926(pinMotorLeftDir, pinMotorLeftPWM, speed);
    setMC33926(pinMotorRightDir, pinMotorRightPWM, speed);
    delay(5);
    for (int i=0; i < 10; i++){
      int sense = 0;
      sense = analogRead(pinMotorLeftSense);
      Serial.print(speed);
      Serial.print(",");    
      Serial.print(sense);
      Serial.print(",");    
      Serial.print(odometryLeft);    
      Serial.print(",");    
      Serial.print(odometryRight);
      Serial.print(",");    
      Serial.print(HIGH);    
      Serial.println();       
    }    
  } 
}

void beschleunigung_z() {
    for (int speed=0; speed >= -125; speed-= 10){    
    setMC33926(pinMotorLeftDir, pinMotorLeftPWM, speed);
    setMC33926(pinMotorRightDir, pinMotorRightPWM, speed);
    delay(5);
    for (int i=0; i < 10; i++){
      int sense = 0;
      sense = analogRead(pinMotorLeftSense);
      Serial.print(speed);
      Serial.print(",");    
      Serial.print(sense);
      Serial.print(",");    
      Serial.print(odometryLeft);    
      Serial.print(",");    
      Serial.print(odometryRight);    
      Serial.println();       
    }    
  } 
}

void bremsen_z() {
    for (int speed=-125; speed <= 0; speed += 10){    
    setMC33926(pinMotorLeftDir, pinMotorLeftPWM, speed);
    setMC33926(pinMotorRightDir, pinMotorRightPWM, speed);
    delay(5);
    for (int i=0; i < 10; i++){
      int sense = 0;
      sense = analogRead(pinMotorLeftSense);
      Serial.print(speed);
      Serial.print(",");    
      Serial.print(sense);
      Serial.print(",");    
      Serial.print(odometryLeft);    
      Serial.print(",");    
      Serial.print(odometryRight);    
      Serial.println();       
    }    
  } 
}
