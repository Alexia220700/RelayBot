#include <Servo.h>

Servo gripperServo;
const int gripperPin = 5;

void setup() {
  gripperServo.attach(gripperPin);
}

void loop() {
  // Open gripper (1000-1500μs range - adjust for your gripper)
  gripperServo.writeMicroseconds(1200);
  delay(2000); // Wait 2000 milliseconds (2 seconds)
  
  // Close gripper (1500-2000μs range - adjust for your gripper)
  gripperServo.writeMicroseconds(1800);
  delay(2000); // Wait 2000 milliseconds (2 seconds)
}
