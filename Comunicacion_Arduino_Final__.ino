#include <QTRSensors.h>
#include <NewPing.h>

#define TRIGGER_PIN   A4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN      A5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define NUM_SENSORS   8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   13    // emitter is controlled by digital pin 2
#define infrared      A3

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
QTRSensorsRC qtrrc((unsigned char[]) {2,3, 4, 5, 6, 7, 8, 12},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

char serIn ='B';

void setup(){
   delay(500);
  Serial.begin(57600); // set the data rate in bits per second for serial data transmission
  delay(1000);
  while (Serial.available() > 0) serIn=Serial.read();
}


void loop(){
  //unsigned int position = qtrrc.readLine(sensorValues);
  if(Serial.available() > 0){
   while (serIn != 'A'){
     serIn = Serial.read();
    }
   if (serIn == 'A'){
  qtrrc.read(sensorValues);
  
  for (unsigned char i = 0; i < NUM_SENSORS; i++){
    Serial.print(sensorValues[i]);
    Serial.print(",");
  }
  //Serial.println(); // uncomment this line if you are using raw values
  //Serial.println(position); // comment this line out if you are using raw values
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print(",");
  Serial.println(analogRead(infrared));
   delay(10);
   serIn = 'B';
   }
   
  }
}
