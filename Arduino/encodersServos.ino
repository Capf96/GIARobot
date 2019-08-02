#include <Servo.h>            //Servo library

//PID angle control for Parallax 360 
unsigned long start;
int pinFeedback_left  = A4;
int pinFeedback_right = A5;
int tCycle  = 0;
int valA    = 85;

float tHigh = 0;
float tLow  = 0;
float dc    = 0;
float angle = 0;    //Measured angle from feedback
float dcMin = 2.9;  //From Parallax spec sheet
float dcMax = 97.1; //From Parallax spec sheet
 
void setup() { 
  pinMode(pinFeedback_left, INPUT);
  pinMode(pinFeedback_right, INPUT);
  Serial.begin(9600);
} 
  
void loop() { 
    //mido_angulo(pinFeedback_left);
    mido_angulo(pinFeedback_right);
    Serial.println(angle);
}

float mido_angulo(int pin) {
  while(1) {  //From Parallax spec sheet
    tHigh  = pulseIn(pin, HIGH);
    tLow   = pulseIn(pin, LOW);
    tCycle = tHigh + tLow;

    if ( tCycle > 1000 && tCycle < 1200) {
        break; //valid tCycle;
    }
  }
  dc = (100 * tHigh) / tCycle; //From Parallax spec sheet, you are trying to determine the percentage of the HIGH in the pulse
  angle = ((dc - dcMin) * 360) / (dcMax - dcMin + 1); //From Parallax spec sheet
  if (angle < 0.0) {
      return;
      angle = 0.0;
  }
  else 
  if (angle > 359.0) {
      return;
      angle = 359.0;
  }
  return angle;
}
