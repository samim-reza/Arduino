#include <WiFi.h>
#include <WebServer.h>

// Change these to match your WiFi credentials
// const char* ssid = "Hotspot";
// const char* password = "123456789";

const char* ssid = "GUB";
const char* password = "GUB!@#2023";

WebServer server(80);

// GPIO pins for motor control
static const uint8_t pwm_A = 26;
static const uint8_t pwm_B = 27;
static const uint8_t dir_A = 32;
static const uint8_t dir_B = 33;

int motor_speed = 255;

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

  // Set motor pins
  pinMode(pwm_A, OUTPUT);
  pinMode(pwm_B, OUTPUT);
  pinMode(dir_A, OUTPUT);
  pinMode(dir_B, OUTPUT);

  // Set up routes
  server.on("/move", HTTP_GET, handleMoveRequest);
  server.onNotFound(handleNotFound);

  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handleMoveRequest() {
  if (!server.hasArg("dir")) {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(400, "text/plain", "Direction not specified");
    return;
  }

  String direction = server.arg("dir");
  Serial.print("Direction: ");
  Serial.println(direction);

  if (direction == "F") {
    forward();
  } else if (direction == "B") {
    backward();
  } else if (direction == "L") {
    turn_left();
  } else if (direction == "R") {
    turn_right();
  } else if (direction == "S") {
    stop_motors();
  } else {
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
