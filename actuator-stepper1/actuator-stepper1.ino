#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// WiFi Credentials
const char* ssid = "NODEMCU";
const char* password = "12345678";

IPAddress local_ip(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

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
const int stepsPerRevolution = 300;
const int degreePerStep = 360 / stepsPerRevolution;

void setup() {
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

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
  server.on("/", handle_OnConnect);
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
void handle_OnConnect() {
  server.send(200, "text/html", SendHTML());
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

    if (command == "front") { moveDualMotors(true, false); Serial.println("front"); }
    else if (command == "back") { moveDualMotors(false, true); Serial.println("back"); }
    else if (command == "rotateLeft") { moveDualMotors(true, true); Serial.println("RL"); }
    else if (command == "rotateRight") { moveDualMotors(false, false); Serial.println("RR"); }
    else if (command == "open") { hand_open(); Serial.println("hand open"); }
    else if (command == "close") { hand_close(); Serial.println("hand close"); }

    // Send current degrees back to client
    String degreeStr = "Motor1: " + String(currentDegree1) + "°, Motor2: " + String(currentDegree2) + "°";
    webSocket.sendTXT(num, degreeStr);
  }
}


// DC Motor Functions
void forward() {
  ledcWrite(0, motor_speed); // Set PWM speed on channel 0
  ledcWrite(1, 0);           // Ensure the other motor channel is OFF
  digitalWrite(dir_A, LOW);  // Set direction
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
  digitalWrite(pwm_A, LOW);  // Set direction
  digitalWrite(pwm_B, HIGH); // Ensure correct direction for forward
  Serial.println("Moving forward");
}

void turn_right() {
  ledcWrite(0, motor_speed); // Set PWM speed on channel 0
  ledcWrite(1, 0);           // Ensure the other motor channel is OFF
  digitalWrite(pwm_A, HIGH); // Set direction
  digitalWrite(pwm_B, LOW);  // Ensure correct direction for forward
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
  Serial.println("Opening");

  for (int i = 0; i < 10; i++) {
    ledcWrite(0, 150);  // Set PWM speed on channel 0
    ledcWrite(1, 0);    // Ensure the other motor channel is OFF
    digitalWrite(pwm_C, LOW);  
    digitalWrite(dir_C, HIGH); // Set direction
    delay(25);  // Motor ON for 25ms

    ledcWrite(0, 0);  // Turn off motor
    digitalWrite(dir_C, LOW);
    digitalWrite(pwm_C, LOW);
    delay(45);  // Motor OFF for 25ms
  }

  // Ensure motor is completely OFF after last iteration
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(dir_C, LOW);
  digitalWrite(pwm_C, LOW);
}


void hand_close() {
  Serial.println("Closing");

  for (int i = 0; i < 10; i++) {
    ledcWrite(0, 150);  // Set PWM speed on channel 0
    ledcWrite(1, 0);    // Ensure the other motor channel is OFF
    digitalWrite(pwm_C, HIGH);  
    digitalWrite(dir_C, LOW); // Set direction
    delay(25);  // Motor ON for 25ms

    ledcWrite(0, 0);  // Turn off motor
    digitalWrite(dir_C, LOW);
    digitalWrite(pwm_C, LOW);
    delay(35);  // Motor OFF for 25ms
  }

  // Ensure motor is completely OFF after last iteration
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  digitalWrite(dir_C, LOW);
  digitalWrite(pwm_C, LOW);
}

String SendHTML() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Unified Motor Control</title>
 <style>
    body {
      font-family: Arial, sans-serif;
      background-color: #f4f4f9;
      text-align: center;
      padding: 20px;
    }
    h1 {
      color: #333;
      margin-bottom: 20px;
      font-size: 2.5vw;
    }
    h2 {
      color: #555;
      margin-top: 30px;
      margin-bottom: 15px;
      font-size: 2vw;
    }
    
    /* Button Styling */
    .button-container {
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      gap: 15px;
    }

    button {
      font-size: 1.5vw;
      padding: 1vw 2vw;
      cursor: pointer;
      border: none;
      border-radius: 8px;
      color: #fff;
      background-color: #007BFF;
      transition: background-color 0.3s, transform 0.2s;
      min-width: 150px;
    }

    button:hover {
      background-color: #0056b3;
    }

    button:active {
      transform: scale(0.95);
    }

    .stop {
      background-color: #FF0000;
    }

    .stop:hover {
      background-color: #b30000;
    }

    footer {
      margin-top: 40px;
      color: #888;
      font-size: 1.2vw;
    }

    /* Responsive Design */
    @media (max-width: 768px) {
      h1 {
        font-size: 6vw;
      }
      h2 {
        font-size: 4vw;
      }
      button {
        font-size: 4vw;
        padding: 4vw;
        width: 80%;
      }
    }
  </style>
</head>
<body>
  <h1>Unified Motor Control</h1>

  <h2>Actuator Motors</h2>
  <button onclick="sendHTTPCommand('F')">Forward</button>
  <button onclick="sendHTTPCommand('B')">Backward</button>
  <button class="stop" onclick="sendHTTPCommand('S')">Stop</button>
  <button onclick="sendHTTPCommand('L')">Hand Up</button>
  <button onclick="sendHTTPCommand('R')">Hand Down</button>

  <h2>Stepper Motors</h2>
  <button onmousedown="sendWSCommand('front')" onmouseup="stopWSCommand()" onmouseleave="stopWSCommand()">Gripper Up</button>
  <button onmousedown="sendWSCommand('back')" onmouseup="stopWSCommand()" onmouseleave="stopWSCommand()">Gripper Down</button>
  <button onmousedown="sendWSCommand('rotateLeft')" onmouseup="stopWSCommand()" onmouseleave="stopWSCommand()">Rotate Left</button>
  <button onmousedown="sendWSCommand('rotateRight')" onmouseup="stopWSCommand()" onmouseleave="stopWSCommand()">Rotate Right</button>

  <h2>Gripper Control</h2>
  <button onmousedown="sendWSCommand('open')" onmouseup="stopWSCommand()" onmouseleave="stopWSCommand()">Open Hand</button>
  <button onmousedown="sendWSCommand('close')" onmouseup="stopWSCommand()" onmouseleave="stopWSCommand()">Close Hand</button>

  <script>
    const serverIP = 'http://' + window.location.hostname;
    const ws = new WebSocket('ws://' + window.location.hostname + ':81');
    let wsInterval;

    function sendHTTPCommand(command) {
      fetch(`${serverIP}/move?dir=${command}`)
        .then(response => {
          if (!response.ok) throw new Error('HTTP Error');
          return response.text();
        })
        .then(console.log)
        .catch(console.error);
    }

    function sendWSCommand(command) {
      ws.send(command);
      wsInterval = setInterval(() => ws.send(command), 3500);
    }

    function stopWSCommand() {
      clearInterval(wsInterval);
    }
  </script>

  <footer>
    &copy; 2025 Unified Motor Control System. All Rights Reserved.
  </footer>
</body>
</html>
)rawliteral";
  return html;
}