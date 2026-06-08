// === LIBRARIES ===
#include <Servo.h>

// === PIN NUMBERS ===
Servo gripperServo;
const int gripperPin = 7;
const int echoPin = 13;
const int trigPin = 12;

// Motor pins
const int leftWheelForward = 9;
const int leftWheelBackward = 6;
const int rightWheelForward = 10;
const int rightWheelBackward = 4;

// ENA/ENB pins
const int ENA = 5;
const int ENB = 11;

// Encoder pins
const int encoderLeft = 3;   // R2
const int encoderRight = 2;  // R1

// Line Sensors
const int lineSensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensorValues[8];
bool sensorOnLine[8];
int lastLinePosition = 0;  // weighted position of the line

// === VARIABLES ===
long duration;
int distance;
bool gripperHasClosed = false;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Flag timing
unsigned long programStartTime = 0;
bool hasStarted = false;
bool isFlagDetected = false;
const unsigned long MAX_WAIT_TIME = 15000;

// Search state tracking
bool searchingForLine = false;
unsigned long searchStartTime = 0;
int searchDirection = 0;  // -1 = left, 1 = right, 0 = not set

int lastError = 0;          // For derivative calculation
float Kp = 0.20;            // Proportional gain (already in your code)
float Kd = 0.05;            // Derivative gain (new)

// === THRESHOLD ===
// CHANGE HERE
const int LINE_THRESHOLD = 850;  // Below = black, above = white

// === SPEED ===
const int BASE_SPEED = 90;
const int TURN_SPEED = 100;
const int HARD_TURN_SPEED = 40;

// Search parameters - UPDATED
const int SEARCH_SPEED = 45;        // Increased for proper turning
const int SEARCH_PULSE_MS = 70;    // Increased to actually rotate
const int SEARCH_PAUSE_MS = 50;     // Shorter pause
const int MAX_SEARCH_TIME = 10000;  // Stop searching after 10 seconds

// Gripper
const int GRIPPER_OPEN = 1800;
const int GRIPPER_CLOSE = 1400;

// Line-weighting
const int sensorWeights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};


// === SETUP ===
void setup() {
  Serial.begin(9600);

  // Motors
  pinMode(leftWheelForward, OUTPUT);
  pinMode(leftWheelBackward, OUTPUT);
  pinMode(rightWheelForward, OUTPUT);
  pinMode(rightWheelBackward, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Encoders
  pinMode(encoderLeft, INPUT_PULLUP);
  pinMode(encoderRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderLeft), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRight), rightEncoderISR, RISING);

  // Line sensors
  for (int i = 0; i < 8; i++) pinMode(lineSensorPins[i], INPUT);

  // Gripper
  gripperServo.attach(gripperPin);
  gripperServo.writeMicroseconds(GRIPPER_OPEN);

  programStartTime = millis();
  
  Serial.println("=== ROBOT READY ===");
}

// === LOOP ===
void loop() {
  //testThresholds();
  
  // Read sensors
  readAllSensors();
  
  // Check if line is detected
  if (isLineDetected()) {
    // Line found - stop searching and follow it
    if (searchingForLine) {
      Serial.println("Line found! Resuming normal following.");
      searchingForLine = false;
      searchDirection = 0;
    }
    followLineNormal();
  } else {
    // No line detected - start or continue searching
    if (!searchingForLine) {
      Serial.println("Line lost! Determining search direction...");
      searchingForLine = true;
      searchStartTime = millis();
      
      // *** CRITICAL: Determine CORRECT search direction ***
      // If robot was turning LEFT (error < 0), line is to the RIGHT
      // So search RIGHT (opposite direction!)
      // If robot was turning RIGHT (error > 0), line is to the LEFT  
      // So search LEFT (opposite direction!)
      
      if (lastLinePosition < 0) {
        // Robot was following LEFT → line is to the RIGHT
        // Search RIGHT
        searchDirection = 1;  // Right
        Serial.println("Was following LEFT → Searching RIGHT");
      } else if (lastLinePosition > 0) {
        // Robot was following RIGHT → line is to the LEFT
        // Search LEFT
        searchDirection = -1; // Left
        Serial.println("Was following RIGHT → Searching LEFT");
      } else {
        // Was centered - default to left
        searchDirection = -1; // Left
        Serial.println("Was centered → Searching LEFT (default)");
      }
    }
    searchForLine();
  }
  
  delay(50);
}


// === FUNCTIONS ===

// Encoder Interrupts
void leftEncoderISR() {
  leftEncoderCount++;
}

void rightEncoderISR() {
  rightEncoderCount++;
}


// Sensor Reading
void readAllSensors() {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(lineSensorPins[i]);
    // CHANGE HERE FOR BLACK LINE
    sensorOnLine[i] = (sensorValues[i] > LINE_THRESHOLD);
  }
}

int calculateLinePosition() {
  long sumWeighted = 0;
  long count = 0;

  for (int i = 0; i < 8; i++) {
    if (sensorOnLine[i]) {
      sumWeighted += sensorWeights[i];
      count++;
    }
  }

  if (count == 0) return 9999;  // No line detected

  return sumWeighted / count;
}

bool isLineDetected() {
  for (int i = 0; i < 8; i++) {
    if (sensorOnLine[i]) return true;
  }
  return false;
}

// === NORMAL LINE FOLLOWING ===
void followLineNormal() {
  int error = calculateLinePosition();
  
  // Safety check
  if (error == 9999) {
    return;
  }
  
  lastLinePosition = error;  // Remember last seen position

  // Slow down on sharp turns
  int dynamicSpeed = BASE_SPEED;
  if (abs(error) > 2000) {
    dynamicSpeed = 45;  // Slow down for sharp turns
  }

  // PD Control
  int derivative = error - lastError;
  int correction = (error * Kp) + (derivative * Kd);
  lastError = error;

  // Display sensor values for debugging
  static unsigned long lastSensorPrint = 0;
  if (millis() - lastSensorPrint > 200) {
    lastSensorPrint = millis();
    Serial.print("Following | Error: ");
    Serial.print(error);
    Serial.print(" | Derivative: ");
    Serial.print(derivative);
    Serial.print(" | Correction: ");
    Serial.print(correction);
    Serial.print(" | Direction: ");
    Serial.print(error > 0 ? "RIGHT" : (error < 0 ? "LEFT" : "CENTER"));
    Serial.print(" | lastLinePosition: ");
    Serial.print(lastLinePosition);
    Serial.print(" | Sensors: ");
    for (int i = 0; i < 8; i++) {
      Serial.print(sensorOnLine[i] ? "1" : "0");
    }
    Serial.println();
  }

  // Calculate motor speeds
  int leftSpeed  = constrain(dynamicSpeed + correction, 0, 255);
  int rightSpeed = constrain(dynamicSpeed - correction, 0, 255);

  // Set motor directions and speeds
  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);
  digitalWrite(leftWheelForward, HIGH);
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, HIGH);
  digitalWrite(rightWheelBackward, LOW);
}


void stopMotors() {
  // Stop PWM speed 
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Make sure all direction pins are LOW
  digitalWrite(leftWheelForward, LOW);
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, LOW);
  digitalWrite(rightWheelBackward, LOW);
}

void searchForLine() {
  // Timeout check
  if (millis() - searchStartTime > MAX_SEARCH_TIME) {
    Serial.println("Search timeout - stopping motors.");
    stopMotors();
    searchingForLine = false;
    searchDirection = 0;
    return;
  }

  // Spot turn based on direction
  if (searchDirection == -1) {
    // Turn LEFT in place
    digitalWrite(leftWheelForward, LOW);
    digitalWrite(leftWheelBackward, HIGH);
    digitalWrite(rightWheelForward, HIGH);
    digitalWrite(rightWheelBackward, LOW);
  } else if (searchDirection == 1) {
    // Turn RIGHT in place
    digitalWrite(leftWheelForward, HIGH);
    digitalWrite(leftWheelBackward, LOW);
    digitalWrite(rightWheelForward, LOW);
    digitalWrite(rightWheelBackward, HIGH);
  }

  // Rotate slowly
  analogWrite(ENA, SEARCH_SPEED);
  analogWrite(ENB, SEARCH_SPEED);

  // Multiple short checks to detect line
  for (int i = 0; i < 5; i++) {
    delay(SEARCH_PULSE_MS / 5);
    readAllSensors();
    if (isLineDetected()) {
      Serial.println("Found line during turn!");
      stopMotors();
      searchingForLine = false;
      searchDirection = 0;
      return;
    }
  }

  stopMotors();
  delay(SEARCH_PAUSE_MS);
  readAllSensors();

  if (isLineDetected()) {
    Serial.println("Found line after turn!");
    searchingForLine = false;
    searchDirection = 0;
  }
}

