#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

const char* ssid = "NODEMCU";
const char* password = "123456789";

// Put IP Address details
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

ESP8266WebServer server(80);

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

const int servo1Pin = D1;
const int servo2Pin = D2;
const int servo3Pin = D3;
const int servo4Pin = D4;
const int servo5Pin = D5;

int upDownPos = 90;
int openClosePos = 90;
int leftRightPos = 180; // Set to middle of 0-360 range
int forwardBackwardPos = 90;
int handRotatePos = 90;

// Set max and min values for specific movements
const int openCloseMin = 0;
const int openCloseMax = 95;
const int upDownMin = 20;
const int upDownMax = 180;
const int forwardBackwardMin = 0;
const int forwardBackwardMax = 180;

void setup() {
  Serial.begin(9600);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);
  servo5.attach(servo5Pin);

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.on("/", handle_OnConnect);
  server.on("/updateServo", handle_updateServo);
  server.on("/setPosition", handle_setPosition);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handle_OnConnect() {
  server.send(200, "text/html", SendHTML(upDownPos, openClosePos, leftRightPos, forwardBackwardPos, handRotatePos));
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
  if (server.hasArg("handRotate")) {
    handRotatePos = server.arg("handRotate").toInt();
    moveServoGradually(servo5, handRotatePos);
  }
  server.send(200, "text/html", SendHTML(upDownPos, openClosePos, leftRightPos, forwardBackwardPos, handRotatePos));
}

void handle_setPosition() {
  int targetUpDownPos, targetOpenClosePos, targetLeftRightPos, targetForwardBackwardPos, targetHandRotatePos;

  if (server.hasArg("position")) {
    String position = server.arg("position");
    if (position == "lowest") {
      targetUpDownPos = upDownMin;
      targetOpenClosePos = openCloseMin;
      targetLeftRightPos = 0;
      targetForwardBackwardPos = forwardBackwardMin;
      targetHandRotatePos = 0;
    } else if (position == "highest") {
      targetUpDownPos = upDownMax;
      targetOpenClosePos = openCloseMax;
      targetLeftRightPos = 360;
      targetForwardBackwardPos = forwardBackwardMax;
      targetHandRotatePos = 180;
    } else if (position == "middle") {
      targetUpDownPos = (upDownMin + upDownMax) / 2;
      targetOpenClosePos = (openCloseMin + openCloseMax) / 2;
      targetLeftRightPos = 180;
      targetForwardBackwardPos = (forwardBackwardMin + forwardBackwardMax) / 2;
      targetHandRotatePos = 90;
    }

    moveServoGradually(servo1, targetUpDownPos);
    servo2.write(targetOpenClosePos);
    moveServoGradually(servo3, map(targetLeftRightPos, 0, 360, 0, 180));
    moveServoGradually(servo4, targetForwardBackwardPos);
    moveServoGradually(servo5, targetHandRotatePos);

    upDownPos = targetUpDownPos;
    openClosePos = targetOpenClosePos;
    leftRightPos = targetLeftRightPos;
    forwardBackwardPos = targetForwardBackwardPos;
    handRotatePos = targetHandRotatePos;
  }
  server.send(200, "text/html", SendHTML(upDownPos, openClosePos, leftRightPos, forwardBackwardPos, handRotatePos));
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

String SendHTML(int upDownPos, int openClosePos, int leftRightPos, int forwardBackwardPos, int handRotatePos) {
  String html = "<!DOCTYPE html>\
<html>\
<head>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\
<title>Servo Control</title>\
<style>\
html { \
    font-family: Helvetica; \
    display: inline-block; \
    margin: 0px auto; \
    text-align: center;\
}\
body {\
    margin-top: 50px;\
}\
h1 {\
    color: #444444;\
    margin: 50px auto 30px;\
}\
h3 {\
    color: #444444;\
    margin-bottom: 50px;\
}\
.slider {\
    width: 300px;\
    margin: 15px auto;\
}\
button {\
    margin: 10px;\
    padding: 10px;\
    font-size: 16px;\
}\
</style>\
<script>\
function updateServo(servo, value) {\
    var xhr = new XMLHttpRequest();\
    xhr.open(\"GET\", \"/updateServo?\" + servo + \"=\" + value, true);\
    xhr.send();\
}\
function setPosition(position) {\
    var xhr = new XMLHttpRequest();\
    xhr.open(\"GET\", \"/setPosition?position=\" + position, true);\
    xhr.send();\
}\
</script>\
</head>\
<body>\
<h1>ESP8266 Servo Control</h1>\
<h3>Using Access Point(AP) Mode</h3>\
<p>Up-Down</p>\
<input type=\"range\" min=\"" + String(upDownMin) + "\" max=\"" + String(upDownMax) + "\" value=\"" + String(upDownPos) + "\" class=\"slider\" id=\"upDown\" oninput=\"updateServo('upDown', this.value)\">\
<p>Open-Close</p>\
<input type=\"range\" min=\"" + String(openCloseMin) + "\" max=\"" + String(openCloseMax) + "\" value=\"" + String(openClosePos) + "\" class=\"slider\" id=\"openClose\" oninput=\"updateServo('openClose', this.value)\">\
<p>Left-Right</p>\
<input type=\"range\" min=\"0\" max=\"360\" value=\"" + String(leftRightPos) + "\" class=\"slider\" id=\"leftRight\" oninput=\"updateServo('leftRight', this.value)\">\
<p>Forward-Backward</p>\
<input type=\"range\" min=\"" + String(forwardBackwardMin) + "\" max=\"" + String(forwardBackwardMax) + "\" value=\"" + String(forwardBackwardPos) + "\" class=\"slider\" id=\"forwardBackward\" oninput=\"updateServo('forwardBackward', this.value)\">\
<br>\
<button onclick=\"setPosition('lowest')\">Lowest</button>\
<button onclick=\"setPosition('highest')\">Highest</button>\
<button onclick=\"setPosition('middle')\">Middle</button>\
</body>\
</html>";
  return html;
}
