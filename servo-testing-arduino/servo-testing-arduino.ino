#include <Servo.h>
Servo armServo;
const int servoPin = 12;
void setup() {
  Serial.begin(9600);
  armServo.attach(servoPin);

}
void loop()
{
  armServo.write(0);
  delay(5000);
  armServo.write(60);
  delay(100);
}