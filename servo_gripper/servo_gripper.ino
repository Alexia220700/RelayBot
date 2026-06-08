#include <Servo.h>

Servo gripperServo;
const int gripperPin = 5;

void setup() {
  gripperServo.attach(gripperPin);
  Serial.begin(9600);
  Serial.println("Gripper Auto-Test Started");
}

void loop() {
  // Test open position
  Serial.println("Opening gripper (1500ms)");
  gripperServo.writeMicroseconds(1500);
  delay(300);
  
  // Test partial close
  Serial.println("Partial close (1600ms)");
  gripperServo.writeMicroseconds(1600);
  delay(300);
  
  // Test full close
  Serial.println("Full close (1800ms)");
  gripperServo.writeMicroseconds(1800);
  delay(300);
  
  // Back to open
  Serial.println("Back to open (1500ms)");
  gripperServo.writeMicroseconds(1500);
  delay(3000);
}
