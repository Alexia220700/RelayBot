// === LIBRARIES ===
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// === NEOPIXEL SETUP ===
#define RED   pixels.Color(255, 0, 0)
#define GREEN pixels.Color(0, 255, 0)
#define OFF   pixels.Color(0, 0, 0)

const int pinNeopixels = 8;
const int numberPixels = 4;
Adafruit_NeoPixel pixels(numberPixels, pinNeopixels, NEO_GRB + NEO_KHZ800);

void setAllPixels(uint32_t color) {
  for (int i = 0; i < numberPixels; i++) pixels.setPixelColor(i, color);
  pixels.show();
}

// === PIN NUMBERS ===
Servo gripperServo;
const int gripperPin = 7;
const int echoPin = 13;
const int trigPin = 12;

const int leftWheelForward = 9;
const int leftWheelBackward = 6;
const int rightWheelForward = 10;
const int rightWheelBackward = 4;
const int ENA = 5;
const int ENB = 11;

const int encoderLeft = 3;
const int encoderRight = 2;

const int lineSensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensorValues[8];
bool sensorOnLine[8];
bool useEight = true;

// === STATE & CONTROL ===
bool bigSquareDetected = false;
bool readyForForcedLeftTurn = false;
bool firstSquareTurnUsed = false;
bool multipleLinesDetected = false;
unsigned long multipleLineStartTime = 0;
const unsigned long MULTI_LINE_TIMEOUT = 200;

int lastGoodLinePosition = 0;
int lastLinePosition = 0;
int consecutiveValidReadings = 0;
const int MIN_VALID_READINGS = 2;

int distanceCm;
bool gripperHasClosed = false;
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

unsigned long programStartTime = 0;
bool isFlagDetected = false;
const unsigned long MAX_WAIT_TIME = 15000;

bool searchingForLine = false;
unsigned long searchStartTime = 0;
int searchDirection = 0;

// PD constants
float Kp = 0.10;
float Kd = 0.05;
int lastError = 0;

const int LINE_THRESHOLD = 800;
const int BASE_SPEED = 90;
const int HARD_TURN_SPEED = 45;
const int MULTI_LINE_SPEED = 60;
const int SEARCH_SPEED = 50;
const int SEARCH_PULSE_MS = 60;
const int SEARCH_PAUSE_MS = 30;
const long MAX_SEARCH_TIME = 5000; // 5 seconds max search

const int GRIPPER_OPEN = 1800;
const int GRIPPER_CLOSE = 1450;
const int CUP_DETECT_CM = 3;

const int sensorWeights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

enum MissionState { WAIT_FLAG, DRIVE_TO_LINE, FOLLOWING_LINE, FINISHED };
MissionState missionState = WAIT_FLAG;
unsigned long lastSeenLineTime = 0;

// =====================================================================
// SETUP & INTERRUPTS
// =====================================================================
void setup() {
  Serial.begin(9600);
  pixels.begin();
  pixels.setBrightness(120);
  setAllPixels(RED);

  pinMode(leftWheelForward, OUTPUT); pinMode(leftWheelBackward, OUTPUT);
  pinMode(rightWheelForward, OUTPUT); pinMode(rightWheelBackward, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);

  for (int i = 0; i < 8; i++) pinMode(lineSensorPins[i], INPUT);

  gripperServo.attach(gripperPin);
  gripperServo.writeMicroseconds(GRIPPER_OPEN);

  attachInterrupt(digitalPinToInterrupt(encoderLeft), []{leftEncoderCount++;}, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRight), []{rightEncoderCount++;}, RISING);

  programStartTime = millis();
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  updateUltrasonic();
  readAllSensors();

  switch(missionState) {
    case WAIT_FLAG:
      handleWaitFlag();
      break;

    case DRIVE_TO_LINE:
      useEight = true;
      if(isLineDetected()) {
        stopMotors();
        missionState = FOLLOWING_LINE;
        useEight = false;
      } else {
        moveForward(BASE_SPEED / 2);
      }
      break;

    case FOLLOWING_LINE:
      handleBigBlackSquareDetection();
      if(isLineDetected()) {
        lastSeenLineTime = millis();
        followLineNormal();
      } else {
        searchForLine();
      }
      handleCupPickup();
      break;

    case FINISHED:
      stopMotors();
      break;
  }
}

// =====================================================================
// SENSING
// =====================================================================
void updateUltrasonic() {
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Shortened timeout (15ms ~ 250cm) to prevent motor lag
  long dur = pulseIn(echoPin, HIGH, 15000); 
  distanceCm = (dur == 0) ? -1 : (int)(dur * 0.034 / 2.0);
}

void readAllSensors() {
  for(int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(lineSensorPins[i]);
    sensorOnLine[i] = (sensorValues[i] > LINE_THRESHOLD);
  }
}

bool isLineDetected() {
  int activeCount = 0;
  for(int i = 0; i < (useEight ? 8 : 6); i++) {
    if(sensorOnLine[i]) activeCount++;
  }
  return (activeCount >= 1); 
}

// Updated: Detects if multiple lines or an extra-wide line exists
bool checkForMultipleLines() {
  int n = useEight ? 8 : 6;
  int activeCount = 0;
  int segments = 0;
  bool inLine = false;

  for(int i = 0; i < n; i++) {
    if(sensorOnLine[i]) {
      activeCount++;
      if(!inLine) { segments++; inLine = true; }
    } else { inLine = false; }
  }
  // If we see two separate lines OR a line wider than 3 sensors
  return (segments >= 2 || activeCount > 3);
}

int calculateLinePosition() {
  long sumWeighted = 0;
  long count = 0;
  int n = useEight ? 8 : 6;
  
  // Identify the longest contiguous segment (The "Main" Line)
  int bestStart = -1, bestEnd = -1, bestLength = 0;
  int currStart = -1, currLen = 0;

  for(int i = 0; i < n; i++) {
    if(sensorOnLine[i]) {
      if(currStart == -1) currStart = i;
      currLen++;
      if(i == n-1 || !sensorOnLine[i+1]) {
        if(currLen > bestLength) {
          bestLength = currLen; bestStart = currStart; bestEnd = i;
        }
        currStart = -1; currLen = 0;
      }
    }
  }

  if(bestLength > 0) {
    for(int i = bestStart; i <= bestEnd; i++) {
      sumWeighted += sensorWeights[i];
      count++;
    }
    int pos = sumWeighted / count;
    lastGoodLinePosition = pos;
    return pos;
  }
  return 9999; 
}

// =====================================================================
// MOVEMENT & LOGIC
// =====================================================================
void followLineNormal() {
  int error = calculateLinePosition();
  if(error == 9999) return;

  bool multi = checkForMultipleLines();
  int dynamicSpeed = multi ? MULTI_LINE_SPEED : BASE_SPEED;
  if(abs(error) > 2000) dynamicSpeed = HARD_TURN_SPEED;

  int derivative = error - lastError;
  int correction = (int)(error * Kp + derivative * Kd);
  lastError = error;

  drive(dynamicSpeed + correction, dynamicSpeed - correction);
}

void searchForLine() {
  if (!searchingForLine) {
    searchingForLine = true;
    searchStartTime = millis();
    // Use last known position to guess direction
    searchDirection = (lastGoodLinePosition < 0) ? -1 : 1; 
    if (readyForForcedLeftTurn && !firstSquareTurnUsed) {
        searchDirection = -1; // Force left
        firstSquareTurnUsed = true;
        readyForForcedLeftTurn = false;
      }
  }

  if (millis() - searchStartTime > MAX_SEARCH_TIME) {
    stopMotors(); missionState = FINISHED; return;
  }

  // Pivot in search direction
  drive(SEARCH_SPEED * searchDirection, SEARCH_SPEED * -searchDirection);
  
  unsigned long pulse = millis();
  while(millis() - pulse < SEARCH_PULSE_MS) {
    readAllSensors();
    // Leniency Fix: If ANY valid line is found, grab it and stop searching
    if(isLineDetected()) {
      searchingForLine = false;
      return;
    }
  }
  stopMotors();
  delay(SEARCH_PAUSE_MS);
}

void handleBigBlackSquareDetection() {
  int active = 0;
  for(int i=0; i<8; i++) if(sensorOnLine[i]) active++;

  if(active >= 7) { // Almost all sensors black
      if(gripperHasClosed) {
        stopMotors(); delay(500);
        gripperServo.writeMicroseconds(GRIPPER_OPEN);
        gripperHasClosed = false;
        missionState = FINISHED;
      } else {
        readyForForcedLeftTurn = true;
      }
  }
}

void handleCupPickup() {
  if(!gripperHasClosed && distanceCm > 0 && distanceCm <= 10) {
    if(distanceCm <= CUP_DETECT_CM) {
      stopMotors(); delay(200);
      gripperServo.writeMicroseconds(GRIPPER_CLOSE);
      gripperHasClosed = true;
      delay(500);
    }
  }
}

void handleWaitFlag() {
  bool flagNow = (distanceCm > 0 && distanceCm < 60);
  if ((millis() - programStartTime) >= MAX_WAIT_TIME) { missionState = DRIVE_TO_LINE; return; }
  if(flagNow && !isFlagDetected) isFlagDetected = true;
  else if(!flagNow && isFlagDetected) missionState = DRIVE_TO_LINE;
}

void drive(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  digitalWrite(leftWheelForward, left > 0);
  digitalWrite(leftWheelBackward, left < 0);
  digitalWrite(rightWheelForward, right > 0);
  digitalWrite(rightWheelBackward, right < 0);

  analogWrite(ENA, abs(left));
  analogWrite(ENB, abs(right));
  setAllPixels(GREEN);
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  setAllPixels(RED);
}
