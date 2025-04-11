// Stepper motor control for Arduino Uno
const int stepPin = 5;   // Digital pin 5
const int dirPin = 4;    // Digital pin 4
const int enPin = 14;    // Analog pin A0 (digital pin 14)

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);  // Enable the driver (LOW usually enables)
}

void loop() {
  // Rotate in one direction
  digitalWrite(dirPin, HIGH);
  for(int x = 0; x < 800; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);  // Controls speed (lower = faster)
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  delay(1000);  // Pause between rotations
  
  // Rotate in opposite direction
  digitalWrite(dirPin, LOW);
  for(int x = 0; x < 800; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
  }
  delay(1000);
}