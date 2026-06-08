int ledRed = 2;
int ledYellow = 7;
int ledGreen = 8;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  red();
  yellow();
  green();
}

void red(){
  digitalWrite(ledRed, LOW);
  delay(500);
  digitalWrite(ledRed, HIGH);
  delay(500);
}

void yellow(){
  digitalWrite(ledYellow, LOW);
  delay(250);
  digitalWrite(ledYellow, HIGH);
  delay(250);
}

void green(){
  digitalWrite(ledGreen, LOW);
  delay(100);
  digitalWrite(ledGreen, HIGH);
  delay(100);
}