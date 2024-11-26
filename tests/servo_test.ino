#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

void setup() {
  myservo.attach(7);  // attaches the servo on pin 7 to the servo object
}

void loop() {
  myservo.write(120);
}