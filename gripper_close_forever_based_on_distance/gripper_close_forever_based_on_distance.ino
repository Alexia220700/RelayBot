#include <Servo.h>

Servo gripperServo;
const int gripperPin = 7;
const int echoPin = 13;
const int trigPin = 12;

long duration;
int distance;
bool gripperHasClosed = false; // Only close once

const int GRIPPER_OPEN = 1800;
const int GRIPPER_CLOSE = 1500;

void setup() {
  gripperServo.attach(gripperPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  
  gripperServo.writeMicroseconds(GRIPPER_OPEN);
  Serial.println("Gripper started in OPEN position");
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  calculate_distance();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  cup();
  
  delay(100);
}

void calculate_distance(){
  distance = duration * 0.034 / 2;
}

void cup(){
  // Close gripper only once when object is detected
  if (!gripperHasClosed && distance < 5 && distance > 0) {
    gripperServo.writeMicroseconds(GRIPPER_CLOSE);
    gripperHasClosed = true;
    Serial.println("Object detected! Gripper CLOSED permanently");
  }
}
