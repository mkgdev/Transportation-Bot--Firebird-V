#include<Stepper.h>
#define STEPS_PER_MOTOR_REVOLUTION 42
Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION,8,10,9,11);
int sequence[4] = {0,1,3,2};   // 0 = red, 1=blue, 2=green, 3=yellow , 1st position = north , 2nd = east , 3rd = south , 4th = west 
int Steps2Take, incomingval ;
void setup() {
    Serial.begin(9600);
  }

void loop() {
   if(Serial.available()>0){
      incomingval = Serial.read();
        if(incomingval == sequence[0]){				// if incoming color variable is equal to that at north, do not move
              delay(100);
          }
        else if(incomingval == sequence[1]){		// if incoming color variable is equal to that at east, rotate 270 clockwise
              small_stepper.setSpeed(300);
              small_stepper.step(165);
              delay(200);
          }
        else if(incomingval == sequence[2]){		// if incoming color variable is equal to that at south, rotate 180 colckwise
               small_stepper.setSpeed(300);
               small_stepper.step(110);
               delay(200);
          }
        else if(incomingval == sequence[3]){		// if incoming color variable is equal to that at west, move 90 clockwise
               small_stepper.setSpeed(300);
               small_stepper.step(55);
               delay(200);
          }

          											// to regain the initial configuration of rotating structure 
        else if(incomingval == 4){					
              delay(100);
          }
        else if(incomingval == 5){					//if this specific value is received, roate 90
                small_stepper.setSpeed(300);
              small_stepper.step(55);
              delay(200);
          }
        else if(incomingval == 6){
               small_stepper.setSpeed(300);
              small_stepper.step(110);
              delay(200);
          }
        else if(incomingval == 7){
               small_stepper.setSpeed(300);
              small_stepper.step(165);
              delay(200);
          }   
    }
  


}
