// pins for the left wheel
const int leftWheelForward = 6;  
const int leftWheelBackward = 5; 

void setup() {
  // set the pins as output
  pinMode(leftWheelForward, OUTPUT);
  pinMode(leftWheelBackward, OUTPUT);

  // initialize both pins to LOW (stop)
  digitalWrite(leftWheelForward, LOW);
  digitalWrite(leftWheelBackward, LOW);
}

void loop() {
  // move the left wheel forward
  digitalWrite(leftWheelForward, HIGH);
  digitalWrite(leftWheelBackward, LOW);
}


// right wheel backwards 11
// right wheel forward 10
// left wheel backwards 6
// left wheel forward 9
// R1 and R2 2, 3
// gripper 4
// echo 12/13
// trig