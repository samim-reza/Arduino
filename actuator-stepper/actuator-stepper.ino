#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// WiFi Credentials
const char* ssid = "GUB";
const char* password = "GUB!@#2023";

// HTTP Server
WebServer server(80);
// WebSocket Server
WebSocketsServer webSocket(81);

// DC Motor Pins
const uint8_t pwm_A = 26;
const uint8_t pwm_B = 27;
const uint8_t pwm_C = 12;
const uint8_t dir_A = 32;
const uint8_t dir_B = 33;
const uint8_t dir_C = 13;

// Stepper Motor Pins
const int stepPin1 = 5;
const int dirPin1 = 4;
const int enPin1 = 14;

const int stepPin2 = 18;
const int dirPin2 = 19;
const int enPin2 = 21;

const int hand_pin = 34;
const int dir_hand = 35;
const int enPin3 = 23;

// Variables
int motor_speed = 50;
int currentDegree1 = 0;
int currentDegree2 = 0;
const int stepsPerRevolution = 100;
const int degreePerStep = 360 / stepsPerRevolution;

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // DC Motor Pins
  pinMode(pwm_A, OUTPUT);
  pinMode(pwm_B, OUTPUT);
  pinMode(dir_A, OUTPUT);
  pinMode(dir_B, OUTPUT);

  pinMode(pwm_C, OUTPUT);
  pinMode(dir_C, OUTPUT);

  // Stepper Motor Pins
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  digitalWrite(enPin1, LOW); // Enable motor driver 1

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);
  digitalWrite(enPin2, LOW); // Enable motor driver 2

  // HTTP Server Routes
  server.on("/move", HTTP_GET, handleMoveRequest);
  server.onNotFound(handleNotFound);

  // WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Start Servers
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  webSocket.loop();
}

// HTTP Handlers
void handleMoveRequest() {
  if (!server.hasArg("dir")) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(400, "text/plain", "Direction not specified");
    return;
  }

  String direction = server.arg("dir");
  Serial.print("Direction: ");
  Serial.println(direction);

  if (direction == "F") forward();
  else if (direction == "B") backward();
  else if (direction == "L") turn_left();
  else if (direction == "R") turn_right();
  else if (direction == "S") stop_motors();
  else {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(400, "text/plain", "Invalid direction");
    return;
  }

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", "Direction: " + direction);
}

void handleNotFound() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(404, "text/plain", "404: Not Found");
}
// Stepper Motor Functions
void moveDualMotors(bool clockwise1, bool clockwise2, int steps = stepsPerRevolution) {
  digitalWrite(dirPin1, clockwise1);
  digitalWrite(dirPin2, clockwise2);

  float degreesRotated1 = 0;  // Track degrees rotated by motor 1
  float degreesRotated2 = 0;  // Track degrees rotated by motor 2

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(800);

    // Update current degrees
    currentDegree1 += clockwise1 ? degreePerStep : -degreePerStep;
    currentDegree2 += clockwise2 ? degreePerStep : -degreePerStep;
    currentDegree1 = (currentDegree1 + 360) % 360;
    currentDegree2 = (currentDegree2 + 360) % 360;

    // Track degrees rotated
    degreesRotated1 += degreePerStep;
    degreesRotated2 += degreePerStep;

    // Check if a full degree has been completed for either motor
    if (degreesRotated1 >= 1.0 || degreesRotated2 >= 1.0) {
      delay(10);  // 10ms delay after every degree
      degreesRotated1 = 0;  // Reset degrees rotated counter
      degreesRotated2 = 0;
    }
  }
}
// WebSocket Handlers
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String command = String((char*)payload);

    if (command == "front") {moveDualMotors(true, false);
    Serial.println("front");}
    else if (command == "back") {moveDualMotors(false, true);
    Serial.println("back");}
    else if (command == "rotateLeft") {moveDualMotors(true, true);
    Serial.println("RL");}
    else if (command == "rotateRight") {moveDualMotors(false, false);
    Serial.println("RR");}
    else if (command == "open") {hand_open();
    Serial.println("hand open");}
    else if (command == "close") {hand_close();
    Serial.println("hand close");}

    // Send current degrees back to client
    String degreeStr = "Motor1: " + String(currentDegree1) + "°, Motor2: " + String(currentDegree2) + "°";
    webSocket.sendTXT(num, degreeStr);
  }
}

// DC Motor Functions
void forward() {
  ledcWrite(0, motor_speed); // Set PWM speed on channel 0
  ledcWrite(1, 0);           // Ensure the other motor channel is OFF
  digitalWrite(dir_A, LOW); // Set direction
  digitalWrite(dir_B, HIGH); // Ensure correct direction for forward
  Serial.println("Moving forward");
}

void backward() {
  ledcWrite(0, motor_speed); // Set PWM speed on channel 0
  ledcWrite(1, 0);           // Ensure the other motor channel is OFF
  digitalWrite(dir_A, HIGH); // Set direction
  digitalWrite(dir_B, LOW);  // Ensure correct direction for backward
  Serial.println("Moving backward");
}

void turn_left() {
  ledcWrite(0, motor_speed); // Set PWM speed on channel 0
  ledcWrite(1, 0);           // Ensure the other motor channel is OFF
  digitalWrite(pwm_A, LOW); // Set direction
  digitalWrite(pwm_B, HIGH); // Ensure correct direction for forward
  Serial.println("Moving forward");
}

void turn_right() {
  ledcWrite(0, motor_speed); // Set PWM speed on channel 0
  ledcWrite(1, 0);           // Ensure the other motor channel is OFF
  digitalWrite(pwm_A, HIGH); // Set direction
  digitalWrite(pwm_B, LOW); // Ensure correct direction for forward
  Serial.println("Moving forward");
}

void stop_motors() {
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(dir_A, LOW);
  digitalWrite(dir_B, LOW);
  digitalWrite(pwm_A, LOW);
  digitalWrite(pwm_B, LOW);
  Serial.println("Motors stopped");
}

void hand_open() {
  ledcWrite(0, 150); // Set PWM speed on channel 0
  ledcWrite(1, 0);           // Ensure the other motor channel is OFF
  digitalWrite(pwm_C, LOW); // Set direction
  digitalWrite(dir_C, HIGH); // Ensure correct direction for forward
  Serial.println("opening");
  delay(500);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(dir_C, LOW);
  digitalWrite(pwm_C, LOW);
}

void hand_close() {
  ledcWrite(0, 150); // Set PWM speed on channel 0
  ledcWrite(1, 0);           // Ensure the other motor channel is OFF
  digitalWrite(pwm_C, HIGH); // Set direction
  digitalWrite(dir_C, LOW); // Ensure correct direction for forward
  Serial.println("closing");
  delay(500);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(dir_C, LOW);
  digitalWrite(pwm_C, LOW);
}

