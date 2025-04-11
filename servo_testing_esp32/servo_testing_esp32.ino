#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>  // ✅ Use ESP32-compatible Servo library

const char* ssid = "B113";       // Replace with your WiFi SSID
const char* password = "PasswoD113";   // Replace with your WiFi Password

Servo armServo;
const int servoPin = 12;

WebServer server(80); // Web server runs on port 80

// HTML content
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html>
  <head>
    <title>ESP32 Servo Control</title>
    <style>
      body { font-family: Arial; text-align: center; margin-top: 50px; }
      button { padding: 20px 40px; font-size: 20px; }
    </style>
  </head>
  <body>
    <h1>ESP32 Servo Control</h1>
    <button onclick="fetch('/move')">Move Servo</button>
  </body>
</html>
)rawliteral";

// Move servo to 0°, wait, then 60°
void moveServoSequence() {
  Serial.println("click");
  armServo.write(0);
  delay(100);
  armServo.write(60);
}

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleMove() {
  moveServoSequence();
  server.send(200, "text/plain", "Servo Moved");
}

void setup() {
  Serial.begin(115200);

  // Attach servo with min/max pulse width for ESP32
  armServo.setPeriodHertz(50); // Standard 50Hz
  armServo.attach(servoPin, 500, 2400); // Min and Max pulse width in microseconds

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/move", handleMove);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
