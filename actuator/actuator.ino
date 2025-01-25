#include <WiFi.h>
#include <WebSocketsServer.h>

// Define your motor control pins
const int in_A = 32;
const int in_B = 33;
const int in_C = 25;
const int in_D = 26;

// WiFi credentials
const char *ssid = "Hotspot";
const char *password = "123456789";

// Initialize WebSocket server
WebSocketsServer webSocket = WebSocketsServer(82);

void setup() {
  Serial.begin(115200);

  // Set motor control pins as output and initialize to LOW
  pinMode(in_A, OUTPUT);
  digitalWrite(in_A, LOW);

  pinMode(in_B, OUTPUT);
  digitalWrite(in_B, LOW);

  pinMode(in_C, OUTPUT);
  digitalWrite(in_C, LOW);

  pinMode(in_D, OUTPUT);
  digitalWrite(in_D, LOW);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
  Serial.println("WebSocket server started");

  // Start the WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  webSocket.loop();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.println("Client connected");
      break;
    case WStype_DISCONNECTED:
      Serial.println("Client disconnected");
      break;
    case WStype_TEXT:
      String msg = String((char *)payload);

      if (msg == "UP") {
        Serial.println("Up");
        digitalWrite(in_B, LOW);
        digitalWrite(in_A, HIGH);
        delay(500); // Run motor for 500ms
        digitalWrite(in_A, LOW); // Stop motor
      } else if (msg == "DOWN") {
        Serial.println("Down");
        digitalWrite(in_A, LOW);
        digitalWrite(in_B, HIGH);
        delay(500); // Run motor for 500ms
        digitalWrite(in_B, LOW); // Stop motor
      } else if (msg == "FRONT") {
        Serial.println("Front");
        digitalWrite(in_D, LOW);
        digitalWrite(in_C, HIGH);
        delay(500); // Run motor for 500ms
        digitalWrite(in_C, LOW); // Stop motor
      } else if (msg == "BACK") {
        Serial.println("Back");
        digitalWrite(in_C, LOW);
        digitalWrite(in_D, HIGH);
        delay(500); // Run motor for 500ms
        digitalWrite(in_D, LOW); // Stop motor
      }
      else if (msg == "STOP") {
        Serial.println("Stop");
        digitalWrite(in_A, LOW);
        digitalWrite(in_B, LOW);
        digitalWrite(in_C, LOW);
        digitalWrite(in_D, LOW);
      }
      break;
  }
}
