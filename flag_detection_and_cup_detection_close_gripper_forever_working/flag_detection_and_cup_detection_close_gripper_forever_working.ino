#include <Servo.h>

Servo gripperServo;
const int gripperPin = 5;
const int echoPin = 13;
const int trigPin = 12;

long duration;
int distance;
bool gripperHasClosed = false; // Only close once

// flag
unsigned long programStartTime = 0;   // Time when program started
bool hasStarted = false;
bool hasGrabbedCup = false;
bool isFlagDetected = false;
const unsigned long MAX_WAIT_TIME = 15000; // 15 seconds max wait (with or without flag)

const int GRIPPER_OPEN = 1800;
const int GRIPPER_CLOSE = 1400;

void setup() {
  // put your setup code here, to run once:
  gripperServo.attach(gripperPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  
  gripperServo.writeMicroseconds(GRIPPER_OPEN);
  Serial.println("Gripper started in OPEN position");
  Serial.println("Robot Ready - Will start in 15 seconds max");
  programStartTime = millis(); // Start the overall timer

}
// flag & cup
void calculate_distance(){
  distance = duration * 0.034 / 2;
}

void loop() {
  // put your main code here, to run repeatedly:
  // flag
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  calculate_distance();
  
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm | ");

  if (!hasStarted) {
    checkFlag();
  } else {
    Serial.println("Robot is moving forward!");
    // Add your motor movement code here
  }

  // cup
  cup();
  
  delay(500);
}

// flag
void checkFlag(){
  unsigned long currentTime = millis();
  unsigned long totalElapsed = currentTime - programStartTime;
  
  // Check if flag is currently detected
  bool flagNow = (distance < 80 && distance > 0);
  
  // If maximum wait time reached, start regardless
  if (totalElapsed >= MAX_WAIT_TIME) {
    Serial.println("Maximum wait time reached! Moving forward");
    hasStarted = true;
    return;
  }
  
  if (flagNow && !isFlagDetected) {
    // Flag just appeared
    isFlagDetected = true;
    Serial.println("Flag detected!");
  }
  else if (!flagNow && isFlagDetected) {
    // Flag just disappeared - start immediately!
    isFlagDetected = false;
    Serial.println("Flag removed! Moving forward immediately");
    hasStarted = true;
  }
  else if (!flagNow) {
    // No flag detected - show time remaining
    unsigned long secondsLeft = (MAX_WAIT_TIME - totalElapsed) / 1000;
    Serial.print("No flag - Starting in ");
    Serial.print(secondsLeft);
    Serial.println(" seconds");
  }
  else {
    // Flag is still detected
    Serial.println("Flag present - will start when removed");
  }
}


// cup
void cup(){
  // Close gripper only once when object is detected
  if (!gripperHasClosed && distance < 5 && distance > 0) {
    gripperServo.writeMicroseconds(GRIPPER_CLOSE);
    gripperHasClosed = true;
    Serial.println("Object detected! Gripper CLOSED permanently");
  }
}
