#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <Servo.h>

const char* ssid = "Hotspot";
const char* password = "123456789";

WebSocketsServer webSocket = WebSocketsServer(81);

Servo upDownServo;
Servo openCloseServo;
Servo leftRightServo;
Servo forwardBackwardServo;

const int upDownServoPin = D1;
const int openCloseServoPin = D2;
const int leftRightServoPin = D3;
const int forwardBackwardServoPin = D8;

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
      Serial.print("Changing openclose: ");
      Serial.println(openClosePos);
      openCloseServo.write(openClosePos);
    } else if (msg.startsWith("leftRight:")) {
      leftRightPos = msg.substring(10).toInt();
      if (leftRightPos < leftRightMin) leftRightPos = leftRightMin;
      if (leftRightPos > leftRightMax) leftRightPos = leftRightMax;
      moveServoGradually(leftRightServo, leftRightPos);
    } else if (msg.startsWith("forwardBackward:")) {
      forwardBackwardPos = msg.substring(16).toInt();
      if (forwardBackwardPos < forwardBackwardMin) forwardBackwardPos = forwardBackwardMin;
      if (forwardBackwardPos > forwardBackwardMax) forwardBackwardPos = forwardBackwardMax;
      Serial.print("Changing for-back: ");
      Serial.println(forwardBackwardPos);
      moveServoGradually(forwardBackwardServo, forwardBackwardPos);
    }
    else if (msg.startsWith("lowest:")) {
      webSocket.sendTXT(num, "0");
      moveServoGradually(upDownServo,80);
      moveServoGradually(leftRightServo,90);
      moveServoGradually(forwardBackwardServo,0);
      Serial.println(upDownPos);
      Serial.println(leftRightPos);
      Serial.println(forwardBackwardPos);
      webSocket.sendTXT(num, "1");
    } else if (msg.startsWith("highest:")) {
      webSocket.sendTXT(num, "0");
      moveServoGradually(upDownServo,0);
      moveServoGradually(leftRightServo,90);
      moveServoGradually(forwardBackwardServo,0);
      webSocket.sendTXT(num, "1");
    } else if (msg.startsWith("middle:")) {
      webSocket.sendTXT(num, "0");
      moveServoGradually(upDownServo,40);
      moveServoGradually(leftRightServo,90);
      moveServoGradually(forwardBackwardServo,0);
      webSocket.sendTXT(num, "1");
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
