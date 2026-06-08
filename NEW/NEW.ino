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

// It expects a 32-bit color value
void setAllPixels(uint32_t color) {
  for (int i = 0; i < numberPixels; i++) pixels.setPixelColor(i, color);
  pixels.show();
}

// === PIN NUMBERS ===
// create a servo object
// tells the arduino there is a servo motor connected
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

// Encoders = measure wheel rotation
const int encoderLeft = 3;
const int encoderRight = 2;

// Line sensors
const int lineSensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
// the amount of light each sensor senses
int sensorValues[8];
// convert the raw numbers into a simple "Yes, I see the line" or "No, I don't."
bool sensorOnLine[8];
int lastLinePosition = 0;
bool useEight = true; // start with 8 sensors

// === FORCED LEFT CONTROL ===
bool bigSquareDetected = false;
bool readyForForcedLeftTurn = false;
bool firstSquareTurnUsed = false;

// === LINE FOLLOWING IMPROVEMENTS ===
bool multipleLinesDetected = false;
unsigned long multipleLineStartTime = 0;
const unsigned long MULTI_LINE_TIMEOUT = 3000; // change if robot follows the wrong stacked line
int lastGoodLinePosition = 0;
int consecutiveValidReadings = 0; // prevents reacting to a single bad reading
const int MIN_VALID_READINGS = 5; // requires 5 consecutive valid readings before accepting a new line position

// === HORIZONTAL LINE COUNTING ===
int horizontalLinesCrossed = 0;
const int REQUIRED_HORIZONTAL_LINES = 3;
unsigned long lastLineCrossTime = 0;
// once the robot sees a line, he ignores other lines for 0.5 seconds
const unsigned long LINE_DEBOUNCE_MS = 500;

// === VARIABLES ===
int distanceCm;
bool gripperHasClosed = false;

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Flag timing
unsigned long programStartTime = 0;
bool isFlagDetected = false;
const unsigned long MAX_WAIT_TIME = 5000;

// Search state
bool searchingForLine = false;
unsigned long searchStartTime = 0;
int searchDirection = 0;

// PD control
int lastError = 0;
float Kp = 0.08;
float Kd = 0.05;

// === THRESHOLD ===
const int LINE_THRESHOLD = 800;

// === SPEED ===
const int BASE_SPEED = 90;
const int HARD_TURN_SPEED = 40;
const int MULTI_LINE_SPEED = 60; // slower when multiple lines detected

// Search parameters
const int SEARCH_SPEED = 45;
const int SEARCH_PULSE_MS = 70;
const int SEARCH_PAUSE_MS = 50;
const int MAX_SEARCH_TIME = 10000;

// Gripper pulse widths
const int GRIPPER_OPEN = 1800;
const int GRIPPER_CLOSE = 1400;

// Line-weighting
const int sensorWeights[8] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

// Mission states
// enum - create a custom list of words that represent numbers
enum MissionState { WAIT_FLAG, DRIVE_TO_LINE, FOLLOW_VERTICAL_LINE, FOLLOWING_LINE, FINISHED };
MissionState missionState = WAIT_FLAG;

// End line timer
unsigned long lastSeenLineTime = 0;
const unsigned long END_LINE_LOST_MS = 90000;

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

  // INPUT_PULLUP: Configures the pin as input with internal pull-up resistor
  // Prevents floating values (random HIGH/LOW fluctuations) when encoder signal is idle
  pinMode(encoderLeft, INPUT_PULLUP);
  pinMode(encoderRight, INPUT_PULLUP);

  // digitalPinToInterrupt(): Converts pin number to interrupt number (Arduino-specific)
  // leftEncoderISR: The Interrupt Service Routine function to call when interrupt triggers
  // RISING: Trigger interrupt when signal goes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(encoderLeft), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderRight), rightEncoderISR, RISING);

  for (int i = 0; i < 8; i++) pinMode(lineSensorPins[i], INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Connects the servo object to a specific Arduino pin
  gripperServo.attach(gripperPin);
  gripperServo.writeMicroseconds(GRIPPER_OPEN);

  stopMotors();
  // used in WAIT_FLAG state
  programStartTime = millis();

  Serial.println("Waiting for flag...");
}

// =====================================================================
// MAIN LOOP 
// =====================================================================
void loop() {
  // reads and processes data from ultrasonic sensors
  updateUltrasonic();
  readAllSensors();

  switch(missionState){

    case WAIT_FLAG:
      handleWaitFlag();
      break;

    case DRIVE_TO_LINE:
      useEight = true; // start with 8 sensors
      
      // Count horizontal lines as we cross them
      if(isLineDetected()) {
        if(millis() - lastLineCrossTime > LINE_DEBOUNCE_MS) {
          horizontalLinesCrossed++;
          lastLineCrossTime = millis();
          Serial.print("Crossed horizontal line ");
          Serial.println(horizontalLinesCrossed);
          
          if(horizontalLinesCrossed >= REQUIRED_HORIZONTAL_LINES) {
            // After 3 horizontal lines, switch to vertical line following
            stopMotors();
            delay(300);
            missionState = FOLLOW_VERTICAL_LINE;
            useEight = false; // switch to 6 inner sensors
            Serial.println("Found vertical line - FOLLOW_VERTICAL_LINE");
            lastGoodLinePosition = calculateLinePosition();
          }
        }
      } else {
        moveForward(BASE_SPEED / 2); // slow forward
      }
      break;

    case FOLLOW_VERTICAL_LINE:
      // Check for first black square on vertical line
      handleFirstBlackSquareDetection(); // NEW FUNCTION
      
      if(isLineDetected()){
        lastSeenLineTime = millis();
        followLineNormal();
      } else {
        searchForLine();
      }

      // Check if it should switch states
      if(readyForForcedLeftTurn && !firstSquareTurnUsed) {
        // First square detected, switch to following line
        missionState = FOLLOWING_LINE;
        Serial.println("First square done → FOLLOWING_LINE");
      }
      break;

    case FOLLOWING_LINE:
      updateUltrasonic();
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

      handleBigBlackSquareDetection();

      if(isLineDetected()){
        lastSeenLineTime = millis();
        followLineNormal();
      } else {
        searchForLine();
      }

      
      break;

    case FINISHED:
      stopMotors();
      break;
  }

}

// =====================================================================
//  Encoder COUNTS
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

  long dur = pulseIn(echoPin, HIGH, 10000);

  // used to convert the raw time signal from an ultrasonic sensor 
  // into a readable distance in centimeters
  distanceCm = (dur == 0) ? -1 : (int)(dur * 0.034 / 2.0);
}

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

void readAllSensors(){
  for(int i = 0; i < 8; i++){
    sensorValues[i] = analogRead(lineSensorPins[i]);
    sensorOnLine[i] = (sensorValues[i] > LINE_THRESHOLD);
  }
}

bool isLineDetected(){
  int n = useEight ? 8 : 6;
  int activeCount = 0;
  int firstActive = -1; // index of the first sensor
  int lastActive = -1; // index of the last sensor
  
  for(int i = 0; i < n; i++){
    if(sensorOnLine[i]) {
      // how many sensors see black
      activeCount++;
      // index of the first sensor that sees black
      if(firstActive == -1) firstActive = i;
      lastActive = i;
    }
  }
  
  // Requires at least 2 adjacent sensors for valid line detection
  if(activeCount < 2) return false;
  
  // Checks if sensors form a mostly contiguous pattern
  // Allows small gaps (1 sensor) but not large gaps
  int gapCount = 0;
  bool inGap = false;
  for(int i = firstActive; i <= lastActive; i++) {
    if(sensorOnLine[i]) {
      inGap = false;
    } else if(!inGap) {
      gapCount++;
      inGap = true;
    }
  }
  
  return (gapCount <= 1); // Allows at most 1 gap
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
  
  // identifys the main contiguous line segment
  int bestStart = -1;
  int bestEnd = -1;
  int bestLength = 0;
  
  int currentStart = -1;
  int currentLength = 0;
  
  for(int i = 0; i < n; i++) {
    if(sensorOnLine[i]) {
      if(currentStart == -1) {
        currentStart = i; // marks where the new black segment starts
      }
      currentLength++;
      
      // Checks if this is the end of a segment
      // n -1 = If the robot is looking at the very last sensor and it is black, 
      // it can't check the next sensor because there isn't one
      // !sensorOnLine[i+1] = the sensor next to the current one is not on the line
      if(i == n-1 || !sensorOnLine[i+1]) {
        if(currentLength > bestLength) {
          bestLength = currentLength; // choose the higher length of black the sensors found
          bestStart = currentStart;
          bestEnd = i;
        }
        currentStart = -1;
        currentLength = 0;
      }
    }
  }
  
  // Calculates position based on the longest contiguous segment
  if(bestLength >= 2) { // Require at least 2 sensors for valid line
    for(int i = bestStart; i <= bestEnd; i++) {
      sumWeighted += sensorWeights[i];
      count++;
    }
    int calculatedPos = sumWeighted / count;
    
    // Validates the position
    if(isValidLinePosition(calculatedPos, bestStart, bestEnd, bestLength)) {
  // Does not accept VERY wide lines (likely stacked lines or black square)
  if(bestLength >= 4) { // adjust based on sensor spacing
    consecutiveValidReadings = 0;
    return lastGoodLinePosition;
  }
  
  consecutiveValidReadings++;
  if(consecutiveValidReadings >= MIN_VALID_READINGS) {
    lastGoodLinePosition = calculatedPos;
  }
  return calculatedPos;
}
  }
  
  // If no valid segment found, return last good position if there is one
  consecutiveValidReadings = 0;
  if(lastGoodLinePosition != 0) {
    return lastGoodLinePosition;
  }
  
  return 9999; // No line found
}

bool isValidLinePosition(int pos, int segmentStart, int segmentEnd, int segmentLength) {
  // Checks if this position makes sense
  if(pos == 9999) return false;
  
  // Checks segment length for stacked lines
  if(segmentLength > 4 && !bigSquareDetected) {
    // Might be multiple lines stacked
    return false;
  }
  
  // Checks if position is consistent with segment location
  int expectedCenter = (sensorWeights[segmentStart] + sensorWeights[segmentEnd]) / 2;
  // abs turns negative numbers into positive ones
  if(abs(pos - expectedCenter) > 1000) {
    return false;
  }
  
  return true;
}

void followLineNormal(){
  int error = calculateLinePosition();
  
  // Handles multiple lines scenario
  if(checkForMultipleLines()) {
    if(!multipleLinesDetected) {
      multipleLinesDetected = true;
      multipleLineStartTime = millis();
      Serial.println("MULTIPLE LINES DETECTED - Using last known line");
    }
    
    // If multiple lines recently detected, use last good position
    // when reaching the stacked lines, robot drives blindly for 200 ms and avoids the 2nd line
    if(millis() - multipleLineStartTime < MULTI_LINE_TIMEOUT) {
      error = lastGoodLinePosition;
    } else {
      multipleLinesDetected = false;
    }
  } else {
    multipleLinesDetected = false;
  }
  
  if(error == 9999) {
    // No line found, use last good position for a short time
    if(lastGoodLinePosition != 0 && millis() - lastSeenLineTime < 200) {
      error = lastGoodLinePosition;
    } else {
      return; // Really lost the line
    }
  }

  lastLinePosition = error;
  
  // Adjust speed based on situation
  int dynamicSpeed = BASE_SPEED;
  // calculates the absolute value of the error
  // If the robot is far off-center (a hard turn), it switches to a slower, more stable speed to avoid overshooting the line
  if(abs(error) > 2000) {
    dynamicSpeed = HARD_TURN_SPEED;
  } else if(multipleLinesDetected) {
    dynamicSpeed = MULTI_LINE_SPEED; // Slow down for multiple lines
  }

  // calculates how much a robot should "correct" its steering to stay on a line
  // derivative = rate of change
  // It predicts where the robot is headed and helps prevent it from overshooting the line by counteracting rapid movements
  int derivative = error - lastError;
  int correction = (int)(error * Kp + derivative * Kd);

  lastError = error;

  // positive correction = left - pushed forward, right - slowed down, ROBOT PIVOT TO RIGHT
  // negative correction = left - decrease, right - increase, ROBOT PIVOT TO LEFT
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

bool checkForMultipleLines() {
  int n = useEight ? 8 : 6;
  int activeCount = 0;
  int gapCount = 0;
  bool inLine = false;
  
  for(int i = 0; i < n; i++) {
    if(sensorOnLine[i]) {
      activeCount++;
      if(!inLine) {
        inLine = true;
      }
    } else if(inLine) {
      gapCount++;
      inLine = false;
    }
  }
  
  // Multiple lines if multiple segments with gaps
  if(gapCount >= 2) {
    return true;
  }
  
  // check if line is unusually wide (might be stacked lines)
  if(activeCount >= 4) {
    // Check if it's a black square
    if(!allSensorsBlack()) {
      return true; // Wide line but not all black = possibly stacked
    }
  }
  
  return false;
}


// =====================================================================
// BIG BLACK SQUARE - FOR SECOND SQUARE ONLY
// =====================================================================
void handleBigBlackSquareDetection() {
  static bool wasAllBlack = false;
  
  bool currentlyAllBlack = allSensorsBlack();
  
  // Checks if it detected the black square
  if(currentlyAllBlack && !wasAllBlack) {
    // first moment all sensors see black
    wasAllBlack = true;
    
    stopMotors();
    
    // Debug output
    Serial.println("SECOND BLACK SQUARE DETECTED - IMMEDIATE STOP!");
    
    // If carrying a cup, drop it
    if(gripperHasClosed) {
      delay(100); 
      gripperServo.writeMicroseconds(GRIPPER_OPEN);
      delay(350); // Wait for gripper to open
      gripperHasClosed = false;
      Serial.println("Dropped object on black square!");
      missionState = FINISHED; // STOP FOREVER
    }
  }
  // Resets detection when no longer on black
  else if(!currentlyAllBlack) {
    wasAllBlack = false;
  }
}

// =====================================================================
// LOST-LINE SEARCH 
// =====================================================================
void searchForLine() {
  // Checks for forced left turn condition
  bool allWhite = true;
  int n = useEight ? 8 : 6;
  for(int i = 0; i < n; i++) {
    if(sensorOnLine[i]) {
      allWhite = false;
      break;
    }
  }

  if (readyForForcedLeftTurn && !firstSquareTurnUsed && allWhite) {
    Serial.println("FORCED LEFT TURN EXECUTING…");
    searchingForLine = true;
    searchStartTime = millis();
    searchDirection = -1;  // left
    
    // Executes a smooth left turn
    for(int i = 0; i < 15; i++) {  // Turn for 15 cycles
      digitalWrite(leftWheelForward, LOW);
      digitalWrite(leftWheelBackward, HIGH);
      digitalWrite(rightWheelForward, HIGH);
      digitalWrite(rightWheelBackward, LOW);
      analogWrite(ENA, 60);
      analogWrite(ENB, 60);
      delay(50);

      readAllSensors();
      if(isLineDetected()) {
        stopMotors();
        searchingForLine = false;
        firstSquareTurnUsed = true;
        readyForForcedLeftTurn = false;
        lastSeenLineTime = millis();
        Serial.println("Found line after forced left turn");
        return;
      }
    }
    stopMotors();
    firstSquareTurnUsed = true;
    readyForForcedLeftTurn = false;
    searchingForLine = false;
    Serial.println("Completed forced left turn");
    return;
  }
  else if (!searchingForLine) {
    searchingForLine = true;
    searchStartTime = millis();
    
    // When multiple lines were detected, search in the direction
    // that would find the original line
    if(multipleLinesDetected && lastGoodLinePosition != 0) {
      searchDirection = (lastGoodLinePosition < 0) ? -1 : 1;
      Serial.print("Multi-line recovery search dir: ");
    } else {
      searchDirection = (lastLinePosition < 0) ? 1 : -1;
      Serial.print("Normal search direction = ");
    }
    Serial.println(searchDirection);
  }

  if (millis() - searchStartTime > MAX_SEARCH_TIME) {
    stopMotors();
    searchingForLine = false;
    Serial.println("Search timeout!");
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
      // Additional check: make sure it's a valid single line
      if(!checkForMultipleLines()) {
        stopMotors();
        searchingForLine = false;
        lastSeenLineTime = millis();
        multipleLinesDetected = false;
        Serial.println("Found valid line, resuming");
        return;
      } else {
        Serial.println("Found multiple lines, continuing search");
      }
    }
  }

  stopMotors();
  delay(SEARCH_PAUSE_MS);
  readAllSensors();
  if (isLineDetected() && !checkForMultipleLines()) {
    searchingForLine = false;
    lastSeenLineTime = millis();
    multipleLinesDetected = false;
  }
}

// =====================================================================
// WAIT FOR FLAG
// =====================================================================
void handleWaitFlag() {
  bool flagNow = (distanceCm > 0 && distanceCm < 30);
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
}

// =====================================================================
// FIRST BLACK SQUARE DETECTION (using vertical line)
// =====================================================================
void handleFirstBlackSquareDetection() {
  static unsigned long firstBlackStartTime = 0;
  static bool wasAllBlack = false;
  
  bool currentlyAllBlack = allSensorsBlack();
  
  if(currentlyAllBlack) {
    if(!wasAllBlack) {
      // Just entered black area on vertical line
      firstBlackStartTime = millis();
      wasAllBlack = true;
    } else {
      // Still on black area
      unsigned long timeOnBlack = millis() - firstBlackStartTime;
      
      // Require longer time for confirmation (100ms)
      if(!bigSquareDetected && timeOnBlack >= 80) {
        bigSquareDetected = true;
        Serial.println("FIRST BLACK SQUARE CONFIRMED on vertical line");
        
        // Continues forward until white, then forced left
        readyForForcedLeftTurn = true;
        Serial.println("Will turn left after leaving black square");
      }
    }
  } else {
  // Left black area - check if it needs to force left turn
    if(wasAllBlack && readyForForcedLeftTurn) {
      stopMotors();
      Serial.println("Left first black square - should turn left now");
      
    }
    wasAllBlack = false;
    bigSquareDetected = false;
  }
}


