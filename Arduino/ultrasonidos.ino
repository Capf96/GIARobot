#include <NewPing.h>

#define TRIGGER_PIN1  A0  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN1     A1  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  A2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     A3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN3  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN3     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  Serial.print(sonar1.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print(" ");
  Serial.print(sonar2.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.print(" ");
  Serial.print(sonar3.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println(" ");
}
