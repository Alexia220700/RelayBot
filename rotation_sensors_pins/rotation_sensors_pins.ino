// rotation sensors - MUST use pins 2,3 for interrupts
const int leftRotationSensor = 2;
const int rightRotationSensor = 3;

// DC Motor pins
const int leftWheelForward = 9;  
const int leftWheelBackward = 6;
const int rightWheelForward = 10;
const int rightWheelBackward = 11;  // Using pin 6 instead of 3 to avoid interrupt conflict

// variables for rotation counting
volatile int leftRotationCount = 0;
volatile int rightRotationCount = 0;

void setup() {
  // motor pins
  pinMode(leftWheelForward, OUTPUT);
  pinMode(leftWheelBackward, OUTPUT);
  pinMode(rightWheelForward, OUTPUT);
  pinMode(rightWheelBackward, OUTPUT);
  
  // rotation sensor pins
  pinMode(leftRotationSensor, INPUT_PULLUP);
  pinMode(rightRotationSensor, INPUT_PULLUP);
  
  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(leftRotationSensor), leftRotationISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightRotationSensor), rightRotationISR, CHANGE);
  
  Serial.begin(9600);
  Serial.println("DC Motor Test Started");
  
  // stop all motors initially
  stopAllMotors();
}

void leftRotationISR() {
  leftRotationCount++;
}

void rightRotationISR() {
  rightRotationCount++;
}

void loop() {
  // Test sequence
  Serial.println("Moving LEFT motor forward...");
  moveLeftForward();
  delay(3000);
  
  Serial.println("Moving LEFT motor backward...");
  moveLeftBackward();
  delay(3000);
  
  Serial.println("Moving RIGHT motor forward...");
  moveRightForward();
  delay(3000);
  
  Serial.println("Moving RIGHT motor backward...");
  moveRightBackward();
  delay(3000);
  
  Serial.println("Stopping all motors...");
  stopAllMotors();
  delay(2000);
  
  // Print rotation counts
  Serial.print("Left Rotations: ");
  Serial.print(leftRotationCount);
  Serial.print(" | Right Rotations: ");
  Serial.println(rightRotationCount);
}

// motor control functions
void moveLeftForward() {
  digitalWrite(leftWheelForward, HIGH);
  digitalWrite(leftWheelBackward, LOW);
}

void moveLeftBackward() {
  digitalWrite(leftWheelForward, LOW);
  digitalWrite(leftWheelBackward, HIGH);
}

void moveRightForward() {
  digitalWrite(rightWheelForward, HIGH);
  digitalWrite(rightWheelBackward, LOW);
}

void moveRightBackward() {
  digitalWrite(rightWheelForward, LOW);
  digitalWrite(rightWheelBackward, HIGH);
}

void stopAllMotors() {
  digitalWrite(leftWheelForward, LOW);
  digitalWrite(leftWheelBackward, LOW);
  digitalWrite(rightWheelForward, LOW);
  digitalWrite(rightWheelBackward, LOW);
}