#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

const char* ssid = "GUB";
const char* password = "GUB!@#2023";

ESP8266WebServer server(80);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const int servo1Pin = D1;
const int servo2Pin = D2;
const int servo3Pin = D3;
const int servo4Pin = D4;

int upDownPos = 90;
int openClosePos = 90;
int leftRightPos = 180; // Set to middle of 0-360 range
int forwardBackwardPos = 90;

const int openCloseMin = 0;
const int openCloseMax = 95;
const int upDownMin = 20;
const int upDownMax = 180;
const int forwardBackwardMin = 0;
const int forwardBackwardMax = 180;

void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);

  // server.on("/updateServo", handle_updateServo);
  server.on("/setPosition", handle_setPosition);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handle_updateServo() {
  if (server.hasArg("upDown")) {
    upDownPos = server.arg("upDown").toInt();
    if (upDownPos < upDownMin) upDownPos = upDownMin;
    if (upDownPos > upDownMax) upDownPos = upDownMax;
    moveServoGradually(servo1, upDownPos);
  }
  if (server.hasArg("openClose")) {
    openClosePos = server.arg("openClose").toInt();
    if (openClosePos < openCloseMin) openClosePos = openCloseMin;
    if (openClosePos > openCloseMax) openClosePos = openCloseMax;
    servo2.write(openClosePos);
  }
  if (server.hasArg("leftRight")) {
    leftRightPos = server.arg("leftRight").toInt();
    if (leftRightPos < 0) leftRightPos = 0;
    if (leftRightPos > 360) leftRightPos = 360;
    moveServoGradually(servo3, map(leftRightPos, 0, 360, 0, 180));
  }
  if (server.hasArg("forwardBackward")) {
    forwardBackwardPos = server.arg("forwardBackward").toInt();
    if (forwardBackwardPos < forwardBackwardMin) forwardBackwardPos = forwardBackwardMin;
    if (forwardBackwardPos > forwardBackwardMax) forwardBackwardPos = forwardBackwardMax;
    moveServoGradually(servo4, forwardBackwardPos);
  }
}

void handle_setPosition() {
  int targetUpDownPos, targetOpenClosePos, targetLeftRightPos, targetForwardBackwardPos;

  if (server.hasArg("position")) {
    String position = server.arg("position");
    if (position == "lowest") {
      targetUpDownPos = upDownMin;
      targetOpenClosePos = openCloseMin;
      targetLeftRightPos = 0;
      targetForwardBackwardPos = forwardBackwardMin;
    } else if (position == "highest") {
      targetUpDownPos = upDownMax;
      targetOpenClosePos = openCloseMax;
      targetLeftRightPos = 360;
      targetForwardBackwardPos = forwardBackwardMax;
    } else if (position == "middle") {
      targetUpDownPos = (upDownMin + upDownMax) / 2;
      targetOpenClosePos = (openCloseMin + openCloseMax) / 2;
      targetLeftRightPos = 180;
      targetForwardBackwardPos = (forwardBackwardMin + forwardBackwardMax) / 2;
    }

    moveServoGradually(servo1, targetUpDownPos);
    servo2.write(targetOpenClosePos);
    moveServoGradually(servo3, map(targetLeftRightPos, 0, 360, 0, 180));
    moveServoGradually(servo4, targetForwardBackwardPos);

    upDownPos = targetUpDownPos;
    openClosePos = targetOpenClosePos;
    leftRightPos = targetLeftRightPos;
    forwardBackwardPos = targetForwardBackwardPos;
  }
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
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
