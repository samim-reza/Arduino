#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// const char* ssid = "GUB";
// const char* password = "GUB!@#2023";

// const char* ssid = "Room_506";
// const char* password = "greeN@121";

const char* ssid = "Hotspot";
const char* password = "123456789";

// const char* ssid = "Tanvir WiFi";
// const char* password = "tanvir2020";

// const char* ssid = "Hepnox";
// const char* password = "Hepnox-Password";

ESP8266WebServer server(80);

static const uint8_t pwm_A = 5;
static const uint8_t pwm_B = 4;
static const uint8_t dir_A = 0;
static const uint8_t dir_B = 2;

// Motor speed = [0-1024]
int motor_speed = 100;
bool reconnecting = false;

void setup() {
  Serial.begin(9600);

  // Connect to WiFi network
  connectToWiFi();
  
  pinMode(pwm_A, OUTPUT);
  pinMode(pwm_B, OUTPUT);
  pinMode(dir_A, OUTPUT);
  pinMode(dir_B, OUTPUT);

  server.on("/move", HTTP_GET, handleMoveRequest);
  server.on("/speed", HTTP_GET, handleSpeedRequest);
  server.on("/pwm", HTTP_GET, handlePWM);
  server.onNotFound(handleNotFound);
  server.begin();
  
  Serial.println("HTTP server started");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    if (!reconnecting) {
      reconnecting = true;
      backward();
    }
    connectToWiFi();
  } else {
    if (reconnecting) {
      stop_motors();
      reconnecting = false;
    }
  }
  server.handleClient();
}

void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void handleMoveRequest() {
  if (!server.hasArg("dir")) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Move: undefined");
    return;
  }
  String direction = server.arg("dir");
  String clientIP = server.client().remoteIP().toString();
  
  Serial.print("Client IP: ");
  Serial.println(clientIP);
  Serial.print("Direction: ");
  Serial.println(direction);
  
  if (direction.equals("F")) {
    forward();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Move: forward");
  } else if (direction.equals("B")) {
    backward();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Move: backward");
  } else if (direction.equals("S")) {
    stop_motors();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Move: stop");
  } else if (direction.equals("L")) {
    turn_left();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Turn: Left");
  } else if (direction.equals("R")) {
    turn_right();
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Turn: Right");
  } else {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Move: undefined");
  }
}

void handleSpeedRequest() {
  if (!server.hasArg("value")) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(404, "text/plain", "Speed: undefined");
    return;
  }
  int value = server.arg("value").toInt();
  String clientIP = server.client().remoteIP().toString();
  
  Serial.print("Client IP: ");
  Serial.println(clientIP);
  Serial.print("Speed set to: ");
  Serial.println(value);
  
  speed_control(value);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Speed: " + String(value));
}

void handleNotFound() {
  String clientIP = server.client().remoteIP().toString();
  
  Serial.print("Client IP: ");
  Serial.println(clientIP);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(404, "text/plain", "404: Not found");
}

void stop_motors() {
  analogWrite(pwm_A, 0);
  analogWrite(pwm_B, 0);
  analogWrite(dir_A, 0);
  analogWrite(dir_B, 0);
  Serial.println("Motors stopped");
}

void backward() {
  analogWrite(pwm_A, 0);
  analogWrite(pwm_B, motor_speed);
  analogWrite(dir_A, motor_speed);
  analogWrite(dir_B, 0);
  Serial.println("Moving backward");
}

void forward() {
  analogWrite(pwm_A, motor_speed);
  analogWrite(pwm_B, 0);
  analogWrite(dir_A, 0);
  analogWrite(dir_B, motor_speed);
  Serial.println("Moving forward");
}

void turn_left() {
  analogWrite(pwm_A, 0);
  analogWrite(pwm_B, motor_speed);
  analogWrite(dir_A, 0);
  analogWrite(dir_B, motor_speed);
  Serial.println("Turning left");
}

void turn_right() {
  analogWrite(pwm_A, motor_speed);
  analogWrite(pwm_B, 0);
  analogWrite(dir_A, motor_speed);
  analogWrite(dir_B, 0);
  Serial.println("Turning right");
}

void speed_control(int num) {
  motor_speed = num*3.41;
  Serial.print("New motor speed set to: ");
  Serial.println(motor_speed);
}

void handlePWM() {
  String response = "{";
  response += "\"pwm\": " + String(motor_speed);
  response += "}";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);
}