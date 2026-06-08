// ENCODER IDENTIFICATION TEST
// Upload this, open Serial Monitor, and follow instructions

const int encoderPin1 = 2;  // Try this as encoder A
const int encoderPin2 = 3;  // Try this as encoder B

// left pin 3
const int motorA_fwd = 9;
const int motorA_bwd = 6;
//right pin 2
const int motorB_fwd = 10;
const int motorB_bwd = 11;

volatile long count1 = 0;
volatile long count2 = 0;

void ISR1() { count1++; }
void ISR2() { count2++; }

void setup() {
  Serial.begin(9600);
  
  // Setup encoder pins
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin1), ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPin2), ISR2, RISING);
  
  // Setup motor pins
  pinMode(motorA_fwd, OUTPUT);
  pinMode(motorA_bwd, OUTPUT);
  pinMode(motorB_fwd, OUTPUT);
  pinMode(motorB_bwd, OUTPUT);
  
  Serial.println("=== ENCODER IDENTIFICATION ===");
  Serial.println("We'll spin each motor and see which encoder counts.");
  Serial.println();
}

void loop() {
  Serial.println("\n--- TEST STARTING ---");
  Serial.println("1. About to spin Motor A (LEFT wheel?)");
  Serial.println("   Press any key to continue...");
  while (!Serial.available()) {}
  Serial.read(); // Clear the key
  
  // Reset counts
  count1 = 0;
  count2 = 0;
  
  // Spin Motor A
  Serial.println("Spinning Motor A...");
  analogWrite(motorA_fwd, 150);
  delay(2000);
  analogWrite(motorA_fwd, 0);
  
  Serial.print("Encoder on pin 2 counted: ");
  Serial.println(count1);
  Serial.print("Encoder on pin 3 counted: ");
  Serial.println(count2);
  
  if (count1 > count2) {
    Serial.println("--> Motor A is connected to encoder on PIN 2");
  } else {
    Serial.println("--> Motor A is connected to encoder on PIN 3");
  }
  
  delay(2000);
  
  Serial.println("\n2. About to spin Motor B (RIGHT wheel?)");
  Serial.println("   Press any key to continue...");
  while (!Serial.available()) {}
  Serial.read(); // Clear the key
  
  // Reset counts
  count1 = 0;
  count2 = 0;
  
  // Spin Motor B
  Serial.println("Spinning Motor B...");
  analogWrite(motorB_fwd, 150);
  delay(2000);
  analogWrite(motorB_fwd, 0);
  
  Serial.print("Encoder on pin 2 counted: ");
  Serial.println(count1);
  Serial.print("Encoder on pin 3 counted: ");
  Serial.println(count2);
  
  if (count1 > count2) {
    Serial.println("--> Motor B is connected to encoder on PIN 2");
  } else {
    Serial.println("--> Motor B is connected to encoder on PIN 3");
  }
  
  Serial.println("\n=== RESULT ===");
  Serial.println("Now you know which encoder pin goes with which motor!");
  Serial.println("Update your main code with this information.");
  
  while (true) {} // Stop here
}