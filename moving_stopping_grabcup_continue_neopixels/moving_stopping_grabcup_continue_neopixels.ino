#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// NEOPIXEL SETUP
const int pinNeopixels = 8;      
const int numberPixels = 4;        
Adafruit_NeoPixel pixels(numberPixels, pinNeopixels, NEO_GRB + NEO_KHZ800);

// NEOPIXEL COLOR FUNCTIONS
void setAllPixels(uint32_t color) {
  for(int i = 0; i < numberPixels; i++) {
    pixels.setPixelColor(i, color);
  }
  pixels.show();
}

void setPixelColor(int pixelIndex, uint32_t color) {
  pixels.setPixelColor(pixelIndex, color);
  pixels.show();
}

// Predefined colors
#define RED pixels.Color(255, 0, 0)
#define GREEN pixels.Color(0, 255, 0)


Servo gripperServo;
const int gripperPin = 7;
const int echoPin = 13;
const int trigPin = 12;

long duration;
int distance;
bool gripperHasClosed = false;

// Motor pins
const int leftWheelForward = 9;
const int leftWheelBackward = 6;
const int rightWheelForward = 10;
const int rightWheelBackward = 4;

// ENA/ENB pins - NOW ON CORRECT PINS!
const int ENA = 5;  // Tell me what pin you moved ENA to!
const int ENB = 11;  // Tell me what pin you moved ENB to!

// Encoder pins
const int encoderLeft = 3;   // R2
const int encoderRight = 2;  // R1

// Encoder variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

int currentSpeed = 200;

// Flag timing
unsigned long programStartTime = 0;
bool hasStarted = false;
bool isFlagDetected = false;
const unsigned long MAX_WAIT_TIME = 15000;

const int GRIPPER_OPEN = 1800;
const int GRIPPER_CLOSE = 1400;

// ========== ENCODER INTERRUPTS ==========
void leftEncoderISR() {
  leftEncoderCount++;
}

void rightEncoderISR() {
  rightEncoderCount++;
}

void moveForward(int speedValue) {
  speedValue = constrain(speedValue, 0, 255);
  
  // Set direction
  digitalWrite(leftWheelForward, HIGH);
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, HIGH);
  digitalWrite(rightWheelBackward, LOW);
  
  // CRITICAL: Enable motors with PWM on ENA/ENB
  analogWrite(ENA, speedValue);
  analogWrite(ENB, speedValue);

  // GREEN when moving
  setAllPixels(RED);
}

void stopMotors() {
  // Set direction
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // THEN set direction pins to LOW
  digitalWrite(leftWheelForward, LOW);
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, LOW);
  digitalWrite(rightWheelBackward, LOW);

  // RED when stopped
  setAllPixels(GREEN);

  delay(500);
}


void setup() {
  // Initialize NeoPixels
  pixels.begin();
  pixels.setBrightness(150);  // Adjust brightness (0-255)

  gripperServo.attach(gripperPin);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  
  gripperServo.writeMicroseconds(GRIPPER_OPEN);
  
  // Motor direction pins
  pinMode(leftWheelForward, OUTPUT);
  pinMode(leftWheelBackward, OUTPUT);
  pinMode(rightWheelForward, OUTPUT);
  pinMode(rightWheelBackward, OUTPUT);
  
  // ENA/ENB pins (PWM)
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Encoder pins
  pinMode(encoderLeft, INPUT_PULLUP);
  pinMode(encoderRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderLeft), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRight), rightEncoderISR, RISING);
  
  stopMotors();
  
  Serial.println("=== ROBOT READY ===");
  Serial.print("ENA=pin");
  Serial.print(ENA);
  Serial.print(", ENB=pin");
  Serial.println(ENB);
  Serial.println("Place flag <80cm, remove to start");
  programStartTime = millis();
}

void calculate_distance() {
  distance = duration * 0.034 / 2;
}

void loop() {
  // Ultrasound reading
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  calculate_distance();
  
  // Display info every 500ms
  static unsigned long lastPrint = 0;
  static long lastLeft = 0;
  static long lastRight = 0;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastPrint >= 500) {
    lastPrint = currentTime;
    
    long leftDelta = leftEncoderCount - lastLeft;
    long rightDelta = rightEncoderCount - lastRight;
    lastLeft = leftEncoderCount;
    lastRight = rightEncoderCount;
    
    Serial.print("Dist: ");
    Serial.print(distance);
    Serial.print("cm | Started: ");
    Serial.print(hasStarted);
    Serial.print(" | Enc L: ");
    Serial.print(leftEncoderCount);
    Serial.print(" R: ");
    Serial.print(rightEncoderCount);
    Serial.print(" | Speed L: ");
    Serial.print(leftDelta * 2);  // Pulses per second
    Serial.print(" R: ");
    Serial.print(rightDelta * 2);  // Pulses per second
    Serial.println(" pps");
  }

  // Robot logic
  if (!hasStarted) {
    checkFlag();
    stopMotors();  // Motors disabled via ENA/ENB
  } else {
    Serial.println("MOVING FORWARD!");
    moveForward(currentSpeed);
  }

  // Cup detection
  cup();
  
  delay(50);
}

void checkFlag() {
  unsigned long currentTime = millis();
  unsigned long totalElapsed = currentTime - programStartTime;
  
  bool flagNow = (distance < 80 && distance > 0);
  
  if (totalElapsed >= MAX_WAIT_TIME) {
    Serial.println("15 seconds up! Moving forward");
    hasStarted = true;
    return;
  }
  
  if (flagNow && !isFlagDetected) {
    isFlagDetected = true;
    Serial.println(">>> FLAG DETECTED! <<<");
  } else if (!flagNow && isFlagDetected) {
    isFlagDetected = false;
    Serial.println(">>> FLAG REMOVED! STARTING! <<<");
    hasStarted = true;
  } else if (!flagNow) {
    unsigned long secondsLeft = (MAX_WAIT_TIME - totalElapsed) / 1000;
    Serial.print("No flag - Starting in ");
    Serial.print(secondsLeft);
    Serial.println(" seconds");
  } else {
    Serial.println("Flag present - waiting...");
  }
}

void cup() {
  if (!gripperHasClosed && distance < 3 && distance > 0) {
    Serial.println(">>> CUP DETECTED! GRABBING! <<<");
    stopMotors();
    delay(500);
    gripperServo.writeMicroseconds(GRIPPER_CLOSE);
    gripperHasClosed = true;
    Serial.println(">>> GRIPPER CLOSED! <<<");
  }
}
