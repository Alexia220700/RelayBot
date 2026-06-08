// === LIBRARIES ===
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// === NEOPIXEL SETUP ===
const int pinNeopixels = 8;
const int numberPixels = 4;
Adafruit_NeoPixel pixels(numberPixels, pinNeopixels, NEO_GRB + NEO_KHZ800);

void setAllPixels(uint32_t color) {
  for (int i = 0; i < numberPixels; i++) pixels.setPixelColor(i, color);
  pixels.show();
}

#define RED   pixels.Color(255, 0, 0)
#define GREEN pixels.Color(0, 255, 0)
#define OFF   pixels.Color(0, 0, 0)

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

// ENA/ENB pins (PWM)
const int ENA = 5;
const int ENB = 11;

// Encoders
const int encoderLeft = 3;
const int encoderRight = 2;

// Line sensors
const int lineSensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensorValues[8];
bool sensorOnLine[8];
int lastLinePosition = 0;
bool useEight = true; // start with 8 sensors

// === FORCED LEFT CONTROL ===
bool bigSquareDetected = false;
bool readyForForcedLeftTurn = false;
bool firstSquareTurnUsed = false;

// === VARIABLES ===
long duration;
int distanceCm;
bool gripperHasClosed = false;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Flag timing
unsigned long programStartTime = 0;
bool isFlagDetected = false;
const unsigned long MAX_WAIT_TIME = 15000;

// Search state
bool searchingForLine = false;
unsigned long searchStartTime = 0;
int searchDirection = 0;
unsigned long fullBlackStart = 0;

// PD control
int lastError = 0;
float Kp = 0.20;
float Kd = 0.05;

// === THRESHOLD ===
const int LINE_THRESHOLD = 850;

// === SPEED ===
const int BASE_SPEED = 90;
const int HARD_TURN_SPEED = 40;

// Search parameters
const int SEARCH_SPEED = 45;
const int SEARCH_PULSE_MS = 70;
const int SEARCH_PAUSE_MS = 50;
const int MAX_SEARCH_TIME = 800000;

// Gripper pulse widths
const int GRIPPER_OPEN = 1800;
const int GRIPPER_CLOSE = 1400;

// Line-weighting
const int sensorWeights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

// Mission states
enum MissionState { WAIT_FLAG, DRIVE_TO_LINE, FOLLOWING_LINE, FINISHED };
MissionState missionState = WAIT_FLAG;

// End line timer
unsigned long lastSeenLineTime = 0;
const unsigned long END_LINE_LOST_MS = 70000;

// Cup detection
const int CUP_DETECT_CM = 3;

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(9600);

  pixels.begin();
  pixels.setBrightness(120);
  setAllPixels(RED);

  pinMode(leftWheelForward, OUTPUT);
  pinMode(leftWheelBackward, OUTPUT);
  pinMode(rightWheelForward, OUTPUT);
  pinMode(rightWheelBackward, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(encoderLeft, INPUT_PULLUP);
  pinMode(encoderRight, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderLeft), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRight), rightEncoderISR, RISING);

  for (int i = 0; i < 8; i++) pinMode(lineSensorPins[i], INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  gripperServo.attach(gripperPin);
  gripperServo.writeMicroseconds(GRIPPER_OPEN);

  stopMotors();
  programStartTime = millis();

  Serial.println("=== ROBOT READY ===");
  Serial.println("Waiting for flag...");
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  updateUltrasonic();
  readAllSensors();

  switch(missionState){

    case WAIT_FLAG:
      handleWaitFlag();
      break;

    case DRIVE_TO_LINE:
      useEight = true; // start with 8 sensors
      if(isLineDetected()) {
        stopMotors();
        missionState = FOLLOWING_LINE;
        useEight = false; // switch to 6 inner sensors after line detected
        Serial.println(F("Line detected → FOLLOWING_LINE"));
      } else {
        moveForward(BASE_SPEED / 2); // slow forward
      }
      break;

    case FOLLOWING_LINE:
      handleBigBlackSquareDetection();

      if(isLineDetected()){
        lastSeenLineTime = millis();
        followLineNormal();
      } else {
        searchForLine();
      }

      // cup pickup
      if(!gripperHasClosed && distanceCm > 0 && distanceCm <= 10){
        int slowSpeed = map(distanceCm, 10, 0, BASE_SPEED/2, 0);
        moveForward(slowSpeed);
        
        if(distanceCm <= CUP_DETECT_CM){
          stopMotors(); 
          delay(200);
          gripperServo.writeMicroseconds(GRIPPER_CLOSE);
          gripperHasClosed = true; 
          delay(350);
        }
      }
      break;

    case FINISHED:
      stopMotors();
      break;
  }

}

// =====================================================================
// INTERRUPTS
// =====================================================================
void leftEncoderISR()  { leftEncoderCount++;  }
void rightEncoderISR() { rightEncoderCount++; }

// =====================================================================
// ULTRASONIC
// =====================================================================
void updateUltrasonic(){
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH); 
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long dur = pulseIn(echoPin, HIGH, 30000);

  distanceCm = (dur == 0) ? -1 : (int)(dur * 0.034 / 2.0);
}

// =====================================================================
// MOVEMENT
// =====================================================================
void moveForward(int speedValue){
  speedValue = constrain(speedValue, 0, 255);
  digitalWrite(leftWheelForward, HIGH);  
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, HIGH); 
  digitalWrite(rightWheelBackward, LOW);

  analogWrite(ENA, speedValue); 
  analogWrite(ENB, speedValue);

  setAllPixels(GREEN);
}
void stopMotors(){
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0);

  digitalWrite(leftWheelForward, LOW);  
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, LOW); 
  digitalWrite(rightWheelBackward, LOW);

  setAllPixels(RED);
}

// =====================================================================
// LINE SENSOR FUNCTIONS
// =====================================================================
void readAllSensors(){
  for(int i = 0; i < 8; i++){
    sensorValues[i] = analogRead(lineSensorPins[i]);
    sensorOnLine[i] = (sensorValues[i] > LINE_THRESHOLD);
  }
}

bool isLineDetected(){
  int n = useEight ? 8 : 6; // respect useEight flag
  for(int i = 0; i < n; i++) if(sensorOnLine[i]) return true;
  return false;
}

bool allSensorsBlack(){
  int n = useEight ? 8 : 6;
  for(int i = 0; i < n; i++) if(sensorValues[i] < LINE_THRESHOLD) return false;
  return true;
}

int calculateLinePosition(){
  long sumWeighted = 0;
  long count = 0;
  int n = useEight ? 8 : 6;

  for(int i = 0; i < n; i++){
    if(sensorOnLine[i]){
      sumWeighted += sensorWeights[i];
      count++;
    }
  }
  return count == 0 ? 9999 : sumWeighted / count;
}

// =====================================================================
// PD LINE FOLLOWING
// =====================================================================
void followLineNormal(){
  int error = calculateLinePosition();
  if(error == 9999) return;

  lastLinePosition = error;

  int dynamicSpeed = BASE_SPEED;
  if(abs(error) > 2000) dynamicSpeed = HARD_TURN_SPEED;

  int derivative = error - lastError;
  int correction = (int)(error * Kp + derivative * Kd);

  lastError = error;

  int leftSpeed  = constrain(dynamicSpeed + correction, 0, 255);
  int rightSpeed = constrain(dynamicSpeed - correction, 0, 255);

  digitalWrite(leftWheelForward, HIGH);  
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, HIGH); 
  digitalWrite(rightWheelBackward, LOW);

  analogWrite(ENA, leftSpeed); 
  analogWrite(ENB, rightSpeed);

  setAllPixels(GREEN);
}

// =====================================================================
// BIG BLACK SQUARE
// =====================================================================
void handleBigBlackSquareDetection(){
  
  if(allSensorsBlack()){
    if(fullBlackStart == 0) fullBlackStart = millis();

    if(!bigSquareDetected && (millis() - fullBlackStart > 70)){
      bigSquareDetected = true;
      Serial.println("BIG BLACK SQUARE CONFIRMED");
    
      if(gripperHasClosed){
        stopMotors(); 
        delay(100);
        gripperServo.writeMicroseconds(GRIPPER_OPEN);
        gripperHasClosed = false;
        Serial.println("Dropped object immediately on black square!");
        delay(500);
      } else {
        readyForForcedLeftTurn = true;
        Serial.println("Left big square → forced-left armed");
      }
    }
  } else {
    // **Reset flag when leaving black square**
    bigSquareDetected = false;
    fullBlackStart = 0;
  }
}

// =====================================================================
// LOST-LINE SEARCH
// =====================================================================
void searchForLine() {
  if (readyForForcedLeftTurn && !firstSquareTurnUsed) {
    Serial.println("FORCED LEFT TURN EXECUTING…");
    searchingForLine = true;
    searchStartTime = millis();
    searchDirection = -1;  // left
    firstSquareTurnUsed = true;
    readyForForcedLeftTurn = false;
  }
  else if (!searchingForLine) {
    searchingForLine = true;
    searchStartTime = millis();
    searchDirection = (lastLinePosition < 0) ? 1 : -1;
    Serial.print("Normal search direction = ");
    Serial.println(searchDirection);
  }

  if (millis() - searchStartTime > MAX_SEARCH_TIME) {
    stopMotors();
    searchingForLine = false;
    return;
  }

  if (searchDirection == -1) {
    digitalWrite(leftWheelForward, LOW);
    digitalWrite(leftWheelBackward, HIGH);
    digitalWrite(rightWheelForward, HIGH);
    digitalWrite(rightWheelBackward, LOW);
  } else {
    digitalWrite(leftWheelForward, HIGH);
    digitalWrite(leftWheelBackward, LOW);
    digitalWrite(rightWheelForward, LOW);
    digitalWrite(rightWheelBackward, HIGH);
  }

  analogWrite(ENA, SEARCH_SPEED);
  analogWrite(ENB, SEARCH_SPEED);

  unsigned long pulseStart = millis();
  while (millis() - pulseStart < SEARCH_PULSE_MS) {
    readAllSensors();
    if (isLineDetected()) {
      stopMotors();
      searchingForLine = false;
      lastSeenLineTime = millis();
      return;
    }
    delay(5);
  }

  stopMotors();
  delay(SEARCH_PAUSE_MS);
  readAllSensors();
  if (isLineDetected()) {
    searchingForLine = false;
    lastSeenLineTime = millis();
  }
}

// =====================================================================
// WAIT FOR FLAG
// =====================================================================
void handleWaitFlag() {
  bool flagNow = (distanceCm > 0 && distanceCm < 80);
  unsigned long elapsed = millis() - programStartTime;

  if (elapsed >= MAX_WAIT_TIME) {
    missionState = DRIVE_TO_LINE;
    return;
  }

  if(flagNow && !isFlagDetected){
    isFlagDetected = true;
  }
  else if(!flagNow && isFlagDetected){
    isFlagDetected = false;
    missionState = DRIVE_TO_LINE;
  }

  delay(300);
}
