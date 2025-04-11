const int actuator1A = 4;
const int actuator1B = 5;

unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT = 2000;
bool connected = false;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);

  pinMode(actuator1A, OUTPUT);
  pinMode(actuator1B, OUTPUT);

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

  if (y == 1) arm_down();
  else if (y == -1) arm_up();

  if (x == 0 && y == 0) stop_all();
}

void checkTimeout() {
  if (millis() - lastCmdTime > TIMEOUT && connected) {
    connected = false;
    stop_all();
  }
}


void arm_up() {
  Serial.println("arm_up");
  for (int i = 0; i < 15; i++) { // adjust iteration count as needed
    digitalWrite(actuator1A, HIGH);
    digitalWrite(actuator1B, LOW);
    delay(10); // small delay for smooth motion
  }
  stop_all(); // stop after motion
}

void arm_down() {
  Serial.println("arm_down");
  for (int i = 0; i < 15; i++) { // adjust iteration count as needed
    digitalWrite(actuator1A, LOW);
    digitalWrite(actuator1B, HIGH);
    delay(10); // small delay for smooth motion
  }
  stop_all(); // stop after motion
}



void stop_all() {
  Serial.println("stop_all");
  digitalWrite(actuator1A, LOW);
  digitalWrite(actuator1B, LOW);
}
