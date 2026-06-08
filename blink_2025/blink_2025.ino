int ledRed = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledRed, OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ledRed, LOW);
  delay(500);
  digitalWrite(ledRed, HIGH);
  delay(500);
}
