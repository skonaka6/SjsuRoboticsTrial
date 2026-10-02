#include <Servo.h>

Servo myServo; // Create Servo object

void setup() {
  // put your setup code here, to run once:
myServo.attach(9); // Attach servo to PIN 9 (Digital Pin D9)
}

void loop() {
  // put your main code here, to run repeatedly:
  myServo.write(0); // Rotate to position 0
  delay(1000); // wait 1000 ms
  myServo.write(90); // Rotate to 90
  delay(1000);
  myServo.write(180); // Rotate to 180
  delay(1000);
}
