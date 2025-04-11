#include <Servo.h>

const int actuator1A = 8;
const int actuator1B = 9;
const int actuator2A = 6;
const int actuator2B = 7;
const int zhengkA = 4;
const int zhengkB = 5;
const int laserPin = 13;
const int servoPin = 12;

Servo armServo;
unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT = 2000;
bool connected = false;
bool laserState = false;

bool but4 = false;
bool but5 = false;
bool y1 = false;
bool y2 = false;
bool x1 = false;
bool x2 = false;

const int MOTOR_SPEED = 150;
const int DURATION = 144;  

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);

  pinMode(actuator1A, OUTPUT);
  pinMode(actuator1B, OUTPUT);
  pinMode(actuator2A, OUTPUT);
  pinMode(actuator2B, OUTPUT);
  pinMode(zhengkA, OUTPUT);
  pinMode(zhengkB, OUTPUT);
  pinMode(laserPin, OUTPUT);
  armServo.attach(servoPin);

  Serial.println("ARDUINO_READY");
}

void loop() {
  handleSerial();
  checkTimeout();
}

void handleSerial() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    lastCmdTime = millis();

    if (cmd == "HELLO") {
      Serial.println("ARDUINO_READY");
      return;
    } else if (cmd == "STOP") {
      stop_all();
      return;
    }

    connected = true;
    processCommand(cmd);
  }
  // else stop_all();
}

void processCommand(String cmd) {
  int x, y, z;
  int buttons[9];
  int idx = 0;

  // Parse CSV
  for (int i = 0; i < 11 && idx < cmd.length(); i++) {
    int nextComma = cmd.indexOf(',', idx);
    String token = (nextComma == -1) ? cmd.substring(idx) : cmd.substring(idx, nextComma);
    int val = token.toInt();
    if (i == 0) x = val;
    else if (i == 1) y = val;
    else if (i == 2) z = val;
    else buttons[i - 3] = val;
    idx = (nextComma == -1) ? cmd.length() : nextComma + 1;
  }

  // Movement logic
  if (buttons[0]) { // If button is pressed
  laserState = !laserState; // Toggle state (ON ↔ OFF)
  digitalWrite(laserPin, laserState ? HIGH : LOW); // Update laser
  // delay(200);
  }

  if (buttons[2]) click();

  if (buttons[4]){
    hand_open();
    but4 = true;
  }
  else if (but4 == true){
    // Serial.println(buttons[4]);
    stop_all();
    but4= false;
  }
  if (buttons[5]) {
    hand_close();
    but5 = true;
  }
  else if (but5 == true){
    // Serial.println(buttons[4]);
    stop_all();
    but5= false;
  }

  if (y == 1)
  {
    arm_down();
    y1 = true;
  }
  else if (y1 == true){
    stop_all();
    y1= false;
  }
  if (y == -1){
    arm_up();
    y2 = true;
  }
  else if (y2 == true){
    stop_all();
    y2= false;
  }

  if (x == 1) {
    move_front();
    x1 = true;
  }
  else if (x1 == true){
    stop_all();
    x1= false;
  }
  if (x == -1) {
    move_back();
    x2 = true;
  }
  else if (x2 == true){
    stop_all();
    x2= false;
  }

  // if (x == 0 && y == 0) stop_all();
}

void checkTimeout() {
  if (millis() - lastCmdTime > TIMEOUT && connected) {
    connected = false;
    stop_all();
  }
}

void click() {
  Serial.println("click");
  armServo.write(0);
  delay(100);
  armServo.write(60);
}

void hand_open() {
  Serial.println("hand_open");
  digitalWrite(zhengkA, LOW);
  analogWrite(zhengkB, MOTOR_SPEED);  // PWM speed control
  //delay(DURATION);
  //stop_all();
}

void hand_close() {
  Serial.println("hand_close");
  digitalWrite(zhengkB, LOW);
  analogWrite(zhengkA, MOTOR_SPEED);  // PWM speed control
  //delay(DURATION);
  //stop_all();
}

void arm_up() {
  Serial.println("arm_up");
  digitalWrite(actuator1A, LOW);
  digitalWrite(actuator1B, HIGH);
}

void arm_down() {
  Serial.println("arm_down");
  digitalWrite(actuator1A, HIGH);
  digitalWrite(actuator1B, LOW);
}

void move_front() {
  Serial.println("move_front");
    digitalWrite(actuator2A, HIGH);
    digitalWrite(actuator2B, LOW);
}

void move_back() {
  Serial.println("move_back");
    digitalWrite(actuator2A, LOW);
    digitalWrite(actuator2B, HIGH);
}

void stop_all() {
  Serial.println("stop_all");
  digitalWrite(actuator1A, LOW);
  digitalWrite(actuator1B, LOW);
  digitalWrite(actuator2A, LOW);
  digitalWrite(actuator2B, LOW);
  digitalWrite(zhengkA, LOW);
  digitalWrite(zhengkB, LOW);
  digitalWrite(laserPin, LOW);
}
