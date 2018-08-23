#include "DualVNH5019MotorShield.h"
#include <SoftwareSerial.h>
#include <Servo.h>
SoftwareSerial BT(5,3); 
DualVNH5019MotorShield md;
Servo myServo; 
String state = ""; 
int i=0;
int mode = 0;
void setup()
{
  Serial.begin(9600);
  md.init();
  BT.begin(9600);
  myServo.attach(11);
}

void loop()
{
if(BT.available() > 0) {// Checks whether data is comming from the serial port
  state = BT.readString();};
if(state.length()<4) 
   i = state.toInt(); 
  
  if(state == "Forward")
    mode = 1;
  if(state == "Back")
    mode = 2;
  if(state == "Left")
    mode = 3;
  if(state == "Right")
    mode = 4;
  if(state == "Clean")
    mode = 5;
  if(state == "Setup")
    mode = 6;
  if(state == "Stop")
    mode = 7;
  if(state == "Setup1")
    mode = 8;
  switch (mode) {
  case 1:
    md.setM1Speed(i);
    md.setM2Speed(i);
    break;
  case 2:
   md.setM1Speed(-i);
    md.setM2Speed(-i);
    break;
  case 3:
   md.setM1Speed(i);
    md.setM2Speed(-i);
    break;  
  case 4:
    md.setM1Speed(-i);
    md.setM2Speed(i);
    break;
  case 5:
    md.setM1Speed(30);
    md.setM2Speed(30);
    delay(100);
    md.setM1Speed(-30);
    md.setM2Speed(-30);
    delay(100);
    break;
  case 6:
    myServo.write(180);
    break;
  case 7:
    md.setM1Speed(0);
    md.setM2Speed(0);
    break;
  case 8:
    myServo.write(0);
    break;
}
 Serial.println(state);
  
 
}
