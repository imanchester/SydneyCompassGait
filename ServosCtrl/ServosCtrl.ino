/************************************************************************
 *                     The Sydney Compass-Gait Robot: 
 *  Design and Modeling of an Open Platform for Dynamic Walking Research
/************************************************************************
 *  
 *         Extention/retraction control of the retractable feet
 *  
 *  Australian Centre for Field Robotics
 *  University of Sydney, Australia
 *  
 *  Date created: 01/08/2013     Date modified: 14/09/2017
 *  by A. Mounir BOUDALI
 *
/************************************************************************/

#include <Servo.h>
/* TUNE HERE the positions of the extention and retraction of the feet */
int innerLegExtend = 126;  // servo pwm value for extending inner leg
int innerLegRetract = 40;  // servo pwm value for retracting inner leg
int outerLegExtend = 40;   // servo pwm value for extending outer leg
int outerLegRetract = 126; // servo pwm value for retracting outer leg

Servo servoI;//Inner
Servo servoO;//Outer
int AnapinI = 0;//Pin connected to the DAQ board (servoI)
int AnapinO = 1;//Pin connected to the DAQ board (servoO)
int consI;//control signal from matlab for servoI
int consO;//control signal from matlab for servoO

void setup() // initiation 
{
servoI.attach(10);//connect the Inner servo to D6
servoO.attach(11);//connect the Outer servo to D5
servoI.write(83);//83 half way (10 -- 83 -- 180)
servoO.write(83);//83
delay(1000);
}

void loop() // control loop
{
consI = analogRead(AnapinI);//read the port A0 (control signalL)
consO = analogRead(AnapinO);//read the port A1 (control signalR)
if (consI>700) // inner leg servo HIGH => retracted
    {
     servoI.write(innerLegRetract);
     if(consO>700){ // outer leg servo HIGH => retracted
        servoO.write(outerLegRetract);
      }      
      else if(consO<300){// otherwise outer leg servo LOW => extended
        servoO.write(outerLegExtend);
      }  
      else {;}
     }

 else if(consI<300) // inner leg servo LOW => extended
    {
     servoI.write(innerLegExtend);
     if(consO>700){ //outer leg servo HIGH => retracted
      servoO.write(outerLegRetract);
      }      
      else if(consO<300){ // outer leg servo LOW => extended
      servoO.write(outerLegExtend);
      }  
      else {;}}
 else {;}
}
