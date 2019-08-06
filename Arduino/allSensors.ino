#include <QTRSensors.h>
#include <NewPing.h>
#include <Servo.h>            //Servo library

//PID angle control for Parallax 360 
unsigned long start;
int tCycle  = 0;
int valA    = 85;

float tHigh = 0;
float tLow  = 0;
float dc    = 0;
float angle = 0;    //Measured angle from feedback
float dcMin = 2.9;  //From Parallax spec sheet
float dcMax = 97.1; //From Parallax spec sheet

#define pinFeedback_left A4 // Encoder de servo Izquierdo
#define pinFeedback_right A5 // Encoder de servo Derecho
#define TRIGGER_PIN1  A0  // Ultrasonido Izquierdo
#define ECHO_PIN1     A1  // 
#define TRIGGER_PIN2  A2  // Ultrasonido central
#define ECHO_PIN2     A3  // 
#define TRIGGER_PIN3  12  // Ultrasonido derecho
#define ECHO_PIN3     11  // 
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // Ultrasonido Izquierdo
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // Ultrasonido Central
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE); // Ultrasonido Derecho

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup(){
  pinMode(pinFeedback_left, INPUT);  // Encoder de servo Izquierdo
  pinMode(pinFeedback_right, INPUT); // Encoder de servo Derecho
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){3, 4, 5, 6, 7, 8, 9, 10}, SensorCount);
  qtr.setEmitterPin(2);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  Serial.begin(57600); // 57600
}

void loop(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  for (uint8_t i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.print(position);
  Serial.print(' ');

  Serial.print(sonar1.ping_cm()); // Ultrasonido Izquierdo
  Serial.print(' ');
  Serial.print(sonar2.ping_cm()); // Ultrasonido Central
  Serial.print(' ');
  Serial.println(sonar3.ping_cm()); // Ultrasonido Derecho
  //Serial.print(' ');
  //Serial.print( mido_angulo(pinFeedback_left) );
  //Serial.print(" ");
  //Serial.println( mido_angulo(pinFeedback_right) );
  //delay(250);
}

float medirAangulo(int pin) {
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
