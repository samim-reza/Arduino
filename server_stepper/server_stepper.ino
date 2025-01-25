#include <WebSocketsServer.h>
#include <WiFi.h>

// WiFi Credentials
// const char* ssid = "Hotspot";
// const char* password = "123456789";

const char* ssid = "GUB";
const char* password = "GUB!@#2023";

// Motor 1 pins
const int stepPin1 = 5;
const int dirPin1 = 4;
const int enPin1 = 14;

// Motor 2 pins
const int stepPin2 = 18;
const int dirPin2 = 19;
const int enPin2 = 21;

// Stepper motor settings
int currentDegree1 = 0;                     // Motor 1 degree position
int currentDegree2 = 0;                     // Motor 2 degree position
const int stepsPerRevolution = 200;         // Adjust based on your motor
const int degreePerStep = 360 / stepsPerRevolution; // Degrees per step

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(81);

// Function to move a motor gradually
void moveMotor(int stepPin, int dirPin, int& currentDegree, bool direction, int steps = stepsPerRevolution) {
  digitalWrite(dirPin, direction); // Set direction
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800); // Adjust speed
    digitalWrite(stepPin, LOW);
    delayMicroseconds(800);

    // Update the current degree incrementally
    currentDegree += direction ? degreePerStep : -degreePerStep;
    if (currentDegree >= 360) currentDegree -= 360;
    if (currentDegree < 0) currentDegree += 360;
  }

  // Print the final degree position
  Serial.print("Motor Degree: ");
  Serial.println(currentDegree);
}

// Function to handle dual-motor rotation (e.g., for rotateLeft and rotateRight)
// Function to move two motors simultaneously
void moveDualMotors(bool clockwise1, bool clockwise2, int steps = stepsPerRevolution) {
  digitalWrite(dirPin1, clockwise1); // Set direction for Motor 1
  digitalWrite(dirPin2, clockwise2); // Set direction for Motor 2

  for (int i = 0; i < steps; i++) {
    // Step both motors
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(800); // Adjust speed

    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(800);

    // Update the current degree incrementally for both motors
    currentDegree1 += clockwise1 ? degreePerStep : -degreePerStep;
    currentDegree2 += clockwise2 ? degreePerStep : -degreePerStep;

    // Keep degrees within 0-360 range for Motor 1
    if (currentDegree1 >= 360) currentDegree1 -= 360;
    if (currentDegree1 < 0) currentDegree1 += 360;

    // Keep degrees within 0-360 range for Motor 2
    if (currentDegree2 >= 360) currentDegree2 -= 360;
    if (currentDegree2 < 0) currentDegree2 += 360;
  }

  // Print the final degree positions
  Serial.print("Motor 1 Degree: ");
  Serial.println(currentDegree1);
  Serial.print("Motor 2 Degree: ");
  Serial.println(currentDegree2);
}


void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String command = String((char*)payload);

    if (command == "front") {
      Serial.println("front");
      moveDualMotors(true, false);
    } else if (command == "back") {
      Serial.println("back");
      moveDualMotors(false, true);
    } else if (command == "rotateLeft") {
      Serial.println("RL");
      moveDualMotors(true, true);
    } else if (command == "rotateRight") {
      Serial.println("RR");
      moveDualMotors(false, false);
    }

    // Send the current degrees of both motors back to the client
    String degreeStr = "Motor1: " + String(currentDegree1) + "°, Motor2: " + String(currentDegree2) + "°";
    webSocket.sendTXT(num, degreeStr);
  }
}

void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);

  // Set up motor pins for Motor 1
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  digitalWrite(enPin1, LOW); // Enable motor driver 1

  // Set up motor pins for Motor 2
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enPin2, OUTPUT);
  digitalWrite(enPin2, LOW); // Enable motor driver 2

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  // Handle WebSocket communication
  webSocket.loop();
}
