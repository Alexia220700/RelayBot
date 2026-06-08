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

// === THRESHOLD ===
// CHANGE HERE
const int LINE_THRESHOLD = 800;  // Below = black, above = white

// === SPEED ===
const int BASE_SPEED = 180;
const int TURN_SPEED = 100;
const int HARD_TURN_SPEED = 150;

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
}

// === LOOP ===
void loop() {
  testThresholds();
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


// === LINE FOLLOWING ===
void followLine() {
  readAllSensors();
  int error = calculateLinePosition();

  // If line is LOST => STOP COMPLETELY
  if (error == 9999) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    digitalWrite(leftWheelForward, LOW);
    digitalWrite(leftWheelBackward, LOW);
    digitalWrite(rightWheelForward, LOW);
    digitalWrite(rightWheelBackward, LOW);
    return;
  }

  // Line detected => normal line following
  float Kp = 0.03;
  int correction = error * Kp;

  int leftSpeed  = constrain(BASE_SPEED + correction, 0, 255);
  int rightSpeed = constrain(BASE_SPEED - correction, 0, 255);

  analogWrite(ENA, leftSpeed);
  analogWrite(ENB, rightSpeed);

  digitalWrite(leftWheelForward, HIGH);
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, HIGH);
  digitalWrite(rightWheelBackward, LOW);
}


// Test THRESHOLDS
void testThresholds() {
  Serial.println("=== SENSOR THRESHOLD TEST MODE ===");

  while (true) {
    for (int i = 0; i < 8; i++) {
      int value = analogRead(lineSensorPins[i]);

      Serial.print("S");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(value);
      Serial.print("  ");
    }
    Serial.println();

    delay(150);
  }
}
