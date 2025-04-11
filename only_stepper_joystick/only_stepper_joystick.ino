const int pull1 = 2;
const int dirPin1 = 3;

const int pull2 = 4;
const int dirPin2 = 5;

unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT = 2000;
bool connectionEstablished = false;
int currentDegree1 = 0;
int currentDegree2 = 0;
const int stepsPerRevolution = 100;
const int degreePerStep = 360 / stepsPerRevolution;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);

  pinMode(pull1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(pull2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

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
    }

    Serial.print("CMD: ");
    Serial.println(cmd);

    if (!connectionEstablished) {
      connectionEstablished = true;
      Serial.println("CONNECTION_ACTIVE");
    }

    processCommand(cmd);
  }
}

void moveDualMotors(bool clockwise1, bool clockwise2, int steps = stepsPerRevolution) {
  digitalWrite(dirPin1, clockwise1);
  digitalWrite(dirPin2, clockwise2);

  float degreesRotated1 = 0;
  float degreesRotated2 = 0;

  for (int i = 0; i < steps; i++) {
    digitalWrite(pull1, HIGH);
    digitalWrite(pull2, HIGH);
    delayMicroseconds(800);
    digitalWrite(pull1, LOW);
    digitalWrite(pull2, LOW);
    delayMicroseconds(800);

    currentDegree1 += clockwise1 ? degreePerStep : -degreePerStep;
    currentDegree2 += clockwise2 ? degreePerStep : -degreePerStep;
    currentDegree1 = (currentDegree1 + 360) % 360;
    currentDegree2 = (currentDegree2 + 360) % 360;

    degreesRotated1 += degreePerStep;
    degreesRotated2 += degreePerStep;

    if (degreesRotated1 >= 1.0 || degreesRotated2 >= 1.0) {
      delay(10);
      degreesRotated1 = 0;
      degreesRotated2 = 0;
    }
  }
}

void processCommand(String cmd) {
  int b2 = getValue(cmd, "C", ',');
  int b3 = getValue(cmd, "B3", ',');
  int b6 = getValue(cmd, "B6", ',');
  int b7 = getValue(cmd, "B7", ',');

  Serial.print("Parsed -> B2: "); Serial.print(b2);
  Serial.print(" B3: "); Serial.print(b3);
  Serial.print(" B6: "); Serial.print(b6);
  Serial.print(" B7: "); Serial.println(b7);

  if (b2 == 1) {
    moveDualMotors(true, false);
    Serial.println("front");
  } else if (b3 == 1) {
    moveDualMotors(false, true);
    Serial.println("back");
  } else if (b6 == 1) {
    moveDualMotors(true, true);
    Serial.println("RL");
  } else if (b7 == 1) {
    moveDualMotors(false, false);
    Serial.println("RR");
  } else {
    stop_all();
  }
}

void checkTimeout() {
  if (millis() - lastCmdTime > TIMEOUT) {
    if (connectionEstablished) {
      Serial.println("CONNECTION_LOST");
      connectionEstablished = false;
      stop_all();
    }
  }
}

void stop_all() {
  digitalWrite(pull1, LOW);
  digitalWrite(pull2, LOW);
}

int getValue(String data, String prefix, char terminator) {
  int prefixIndex = data.indexOf(prefix);
  if (prefixIndex == -1) return 0;

  int terminatorIndex = data.indexOf(terminator, prefixIndex);
  if (terminatorIndex == -1) terminatorIndex = data.length();

  return data.substring(prefixIndex + prefix.length(), terminatorIndex).toInt();
}
