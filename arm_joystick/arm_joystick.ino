#include <Servo.h>

const int actuator1A = 8;
const int actuator1B = 9;
const int actuator2A = 6;
const int actuator2B = 7;
const int zhengkA = 4;
const int zhengkB = 5;
const int laserPin = 12;
const int servoPin = 13;

Servo armServo;
unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT = 2000;
bool connected = false;
bool laserState = false;

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

  if (buttons[1]) click();

  if (buttons[4]) hand_open();
  if (buttons[5]) hand_close();

  if (y == 1) arm_down();
  else if (y == -1) arm_up();

  if (x == 1) move_front();
  else if (x == -1) move_back();

  if (x == 0 && y == 0) stop_all();
}

void checkTimeout() {
  if (millis() - lastCmdTime > TIMEOUT && connected) {
    connected = false;
    stop_all();
  }
}

void click() {
  Serial.println("click");
  armServo.write(45);
  delay(300);
  armServo.write(0);
}

void hand_open() {
  Serial.println("hand_open");
    for (int i = 0; i < 12; i++) { // adjust iteration count as needed
    digitalWrite(zhengkB, HIGH);
    digitalWrite(zhengkA, LOW);
    delay(12); // small delay for smooth motion
  }
  stop_all(); // stop after motion
}

void hand_close() {
  Serial.println("hand_close");
    for (int i = 0; i < 12; i++) { // adjust iteration count as needed
    digitalWrite(zhengkB, LOW);
    digitalWrite(zhengkA, HIGH);
    delay(12); // small delay for smooth motion
  }
  stop_all(); // stop after motion
}

void arm_up() {
  Serial.println("arm_up");
  for (int i = 0; i < 12; i++) { // adjust iteration count as needed
    digitalWrite(actuator1A, HIGH);
    digitalWrite(actuator1B, LOW);
    delay(12); // small delay for smooth motion
  }
  stop_all(); // stop after motion
}

void arm_down() {
  Serial.println("arm_down");
  for (int i = 0; i < 12; i++) { // adjust iteration count as needed
    digitalWrite(actuator1A, LOW);
    digitalWrite(actuator1B, HIGH);
    delay(12); // small delay for smooth motion
  }
  stop_all(); // stop after motion
}

void move_front() {
  Serial.println("move_front");
  for (int i = 0; i < 12; i++) { // adjust iteration count as needed
    digitalWrite(actuator2A, HIGH);
    digitalWrite(actuator2B, LOW);
    delay(12); // small delay for smooth motion
  }
  stop_all(); // stop after motion
}

void move_back() {
  Serial.println("move_back");
  for (int i = 0; i < 12; i++) { // adjust iteration count as needed
    digitalWrite(actuator2A, LOW);
    digitalWrite(actuator2B, HIGH);
    delay(10); // small delay for smooth motion
  }
  stop_all(); // stop after motion
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
