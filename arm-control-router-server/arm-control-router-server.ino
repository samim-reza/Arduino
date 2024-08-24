#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Servo.h>

const char* ssid = "Hepnox";
const char* password = "Hepnox-Password";

WebSocketsServer webSocket = WebSocketsServer(81);

Servo upDownServo;
Servo openCloseServo;
Servo leftRightServo;
Servo forwardBackwardServo;

const int upDownServoPin = D1;
const int openCloseServoPin = D2;
const int leftRightServoPin = D3;
const int forwardBackwardServoPin = D4;

int openClosePos = 90;
int upDownPos = 80;
int leftRightPos = 180;
int forwardBackwardPos = 100;

const int openCloseMin = 0;
const int openCloseMax = 95;
const int upDownMin = 0;
const int upDownMax = 80;
const int forwardBackwardMin = 0;
const int forwardBackwardMax = 100;
const int leftRightMin = 0;
const int leftRightMax = 180;

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

  upDownServo.attach(upDownServoPin);
  openCloseServo.attach(openCloseServoPin);
  leftRightServo.attach(leftRightServoPin);
  forwardBackwardServo.attach(forwardBackwardServoPin);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");
}

void loop() {
  webSocket.loop();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    if (msg.startsWith("upDown:")) {
      upDownPos = msg.substring(7).toInt();
      if (upDownPos < upDownMin) upDownPos = upDownMin;
      if (upDownPos > upDownMax) upDownPos = upDownMax;
      Serial.print("Changing up-down: ");
      Serial.println(upDownPos);
      moveServoGradually(upDownServo, upDownPos);
    } else if (msg.startsWith("openClose:")) {
      openClosePos = msg.substring(10).toInt();
      if (openClosePos < openCloseMin) openClosePos = openCloseMin;
      if (openClosePos > openCloseMax) openClosePos = openCloseMax;
      openCloseServo.write(openClosePos);
    } else if (msg.startsWith("leftRight:")) {
      leftRightPos = msg.substring(10).toInt();
      if (leftRightPos < leftRightMin) leftRightPos = leftRightMin;
      if (leftRightPos > leftRightMax) leftRightPos = leftRightMax;
      moveServoGradually(leftRightServo, leftRightPos);
    } else if (msg.startsWith("forwardBackward:")) {
      forwardBackwardPos = msg.substring(15).toInt();
      if (forwardBackwardPos < forwardBackwardMin) forwardBackwardPos = forwardBackwardMin;
      if (forwardBackwardPos > forwardBackwardMax) forwardBackwardPos = forwardBackwardMax;
      moveServoGradually(forwardBackwardServo, forwardBackwardPos);
    } else if (msg.startsWith("position:")) {

      String position = msg.substring(9);

      int targetUpDownPos, targetOpenClosePos, targetLeftRightPos, targetForwardBackwardPos;

      if (position == "lowest") {
        targetUpDownPos = upDownMin;
        targetForwardBackwardPos = forwardBackwardMin;
      } else if (position == "highest") {
        targetUpDownPos = upDownMax;
        targetForwardBackwardPos = forwardBackwardMax;
      } else if (position == "middle") {
        targetUpDownPos = (upDownMin + upDownMax) / 2;
        targetForwardBackwardPos = (forwardBackwardMin + forwardBackwardMax) / 2;
      }

      moveServoGradually(upDownServo, targetUpDownPos);
      openCloseServo.write(targetOpenClosePos);
      moveServoGradually(leftRightServo, leftRightPos);
      moveServoGradually(forwardBackwardServo, targetForwardBackwardPos);

      upDownPos = targetUpDownPos;
      openClosePos = targetOpenClosePos;
      leftRightPos = targetLeftRightPos;
      forwardBackwardPos = targetForwardBackwardPos;
    }
  }
}

void moveServoGradually(Servo& servo, int targetPos) {
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
