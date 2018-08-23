#include "src/mpu.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 Mydisp(OLED_RESET);

int valL = 0;
int valR = 0;
int tL = 0;
int tR = 0;

// motor one
int enA = 10;
int in1 = 9;
int in2 = 8;
// motor two
int enB = 5;
int in3 = 7;
int in4 = 6;

float theta0 = 0;
float x = 0;
float y = 0;
float du = 0;
float dtheta = 0;
int b = 13;
float theta = 0;

int countL = 0;
int countR = 0;
int Lold = 0;
int Rold = 0;
int dL = 0;
int dR = 0;
int tim = 20;

int goal_i = 0;
float goal_theta = 0;
float goal_setx[3]={20,40,0};
float goal_sety[3]={1,14,0};
float goalX = goal_setx[0];
float goalY = goal_sety[0];

int ret;

void setup() {
  Wire.begin();
  ret = mympu_open(200);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(2, INPUT);
  pinMode(3, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.begin(9600);

  Mydisp.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  Mydisp.setTextColor(WHITE);
  Mydisp.setTextSize(1);
}


// the loop function runs over and over again forever

void demoThree()
{
  // this function will run the motors across the range of possible speeds
  // note that maximum speed is determined by the motor itself and the
  //operating voltage
  // the PWM values sent by analogWrite() are fractions of the maximum
  //speed possible
  // by your hardware
  // turn on motors
  //1 low 2 high  left reverse

  //3 low 4 high  right forward
  analogWrite(enA, 100);
  analogWrite(enB, 100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, HIGH);
  // delay(1000);
  // digitalWrite(in1, LOW);
  // digitalWrite(in2, LOW);
  // digitalWrite(in3, LOW);
  // digitalWrite(in4, LOW);
  // delay(1000);
}

void loop() {
  
  if (millis() < 3000) {

    if ( mympu_update() == 0 )
      theta0 = mympu.ypr[0];

  }

  valL = digitalRead(2);

  valR = digitalRead(3);
  if ( mympu_update() == 0 ) 
    theta = (mympu.ypr[0] - theta0) / 180 * 3.14;
    if (millis() / 200 - tim == 1) //
    {
      tim++;
      dL = countL - Lold;
      dR = countR - Rold;

      du = 1.0 * (dL + dR) / 2;
      dtheta = 1.0 * (dR - dL) / b;
      // theta = theta + dtheta;

      x = x + du * cos(theta);
      y = y + du * sin(theta);

      Lold = countL;
      Rold = countR;
    }

    //  Serial.print(x);
    //  Serial.print(",");
    //  Serial.println(y);

    Mydisp.clearDisplay();
    Mydisp.setCursor(20, 5);
    Mydisp.print("X ");
    Mydisp.println(x);
    // default print is decimal format
    Mydisp.setCursor(20, 20);
    Mydisp.print("Y ");
    Mydisp.print(-y);

    Mydisp.display();

    if (tL != valL) {
      tL = valL;
      countL++;
    }
    if (tR != valR) {
      tR = valR;
      countR++;
    }
    goal_theta = atan((goalY - y) / (goalX - x));
    if (millis() > 3000) {
      //turn right
      if ((theta - goal_theta) >= 0.175) {
        Serial.println("right");
        analogWrite(enA, 255);
        analogWrite(enB, 100);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
      }
      //turn left
      if ((theta - goal_theta) <= -0.175) {
                Serial.println("left");
        analogWrite(enA, 100);
        analogWrite(enB, 255);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
      }
      //stright
      if ((theta - goal_theta) < 0.175 && (theta - goal_theta) > -0.175 ) {
                Serial.println("stright");
        analogWrite(enA, 255);
        analogWrite(enB, 255);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);

        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
      }

      if (abs(x - goalX) < 5 && abs(y - goalY) < 5) {
        if(goal_i<2){
        goal_i++;
        goalX = goal_setx[goal_i];
        goalY = goal_sety[goal_i];}
        
        if(goal_i == 2){
        Serial.println("stop");
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);}
      }
    }
  }
