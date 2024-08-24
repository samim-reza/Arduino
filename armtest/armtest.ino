#include <Servo.h>

Servo left;
Servo right; 
Servo hand;

void setup() {
  Serial.begin(9600);
  left.attach(D4);  
  right.attach(D5);  
}

int d = 15;

void loop()
{
  // moveServoGradually(left,0);
  // Serial.println("degree 0");
  // delay(3000);
  // moveServoGradually(left,65);
  // Serial.println("degree 65");
  // delay(3000);
  // moveServoGradually(left,130);
  // Serial.println("degree 130");
  // delay(3000);
  // moveServoGradually(left,65);
  // Serial.println("degree 65");
  // delay(3000);
  // moveServoGradually(left,0);
  // Serial.println("degree 0");
  // delay(3000);

  moveServoGradually(right, 80);
  Serial.println("degree 180");
  delay(3000);
  moveServoGradually(right,40);
  Serial.println("degree 90");
  delay(3000);
  moveServoGradually(right,0);
  Serial.println("degree 0");
  delay(3000);
  moveServoGradually(right,40);
  Serial.println("degree 90");
  delay(3000);
  moveServoGradually(right,80);
  Serial.println("degree 180");

}

void moveServoGradually(Servo &servo, int targetPos) {
  int currentPos = servo.read();
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      servo.write(pos);
      delay(15);
    }
  } else {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      servo.write(pos);
      delay(15);
    }
  }
}