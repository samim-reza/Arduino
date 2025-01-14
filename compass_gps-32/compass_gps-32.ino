#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

//contro speed
int speed = 200;
#define EN1 25
#define EN2 26
// Motor control pins
#define IN1 2
#define IN2 4
#define IN3 5
#define IN4 18

const char* ssid = "Hotspot";         // Replace with your Wi-Fi SSID
const char* password = "123456789";
WebSocketsServer webSocket(81);

// TinyGPS++ and Serial communication for GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // UART1 for GPS

// HMC5883L Magnetometer configuration
#define HMC5883L_Address 0x1E
#define DataRegisterBegin 0x03  // Start of data registers for X, Z, and Y axes

// Current GPS coordinates
float currentLat = 0.0;
float currentLng = 0.0;
float targetLat = 23.8296169;
float targetLng = 90.5672889;

// Distance threshold (e.g., 5 meters)
float distance = 5.0;

// Bearing (direction to target)
float bearing = 0.0;
float currentHeading = 0.0;  // Heading from the magnetometer

WiFiServer server(80);

void setup() {
  // Motor pins setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  analogWrite(EN1,speed);
  analogWrite(EN2,speed);

  // Initialize serial and Wi-Fi
  Serial.begin(9600);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to Wi-Fi. IP: ");
  Serial.println(WiFi.localIP());

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  Serial.println("WebSocket server started on port 81.");
  
  // Initialize GPS serial communication
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

  // Initialize I2C communication for HMC5883L
  Wire.begin(21, 22);  // SDA -> GPIO21, SCL -> GPIO22 for ESP32

  // Configure HMC5883L to continuous measurement mode
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(0x02);  // Mode register
  Wire.write(0x00);  // Set to continuous measurement mode
  Wire.endTransmission();

  delay(100);  // Wait for sensor to initialize
}

void loop() {
  webSocket.loop();
  
  // Process GPS data
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      if (gps.location.isUpdated()) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();

        // Print GPS coordinates
        Serial.print("Latitude: ");
        Serial.println(currentLat, 6);
        Serial.print("Longitude: ");
        Serial.println(currentLng, 6);
      }
    }
  }

  // Read magnetometer data
  readMagnetometer();

  // Calculate bearing and distance if target coordinates are available
  if (targetLat != 0.0 && targetLng != 0.0) {
    calculateBearing();
    if (abs(bearing - currentHeading) < 15) {  // If bearing is almost aligned
      moveForward();
    } else if (distance < 5) {
      stopMotors();
    } else {
      adjustHeading();
    }
  }
  delay(1000);  // Delay to make output readable
}

void onWebSocketEvent(uint8_t client_num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("Client [%u] disconnected.\n", client_num);
      break;

    case WStype_CONNECTED:
      Serial.printf("Client [%u] connected.\n", client_num);
      break;

    case WStype_TEXT:
      // Convert the received payload to a string
      String receivedData = String((char*)payload);

      Serial.printf("Client [%u] sent: %s\n", client_num, receivedData.c_str());

      // Parse the received data as latitude and longitude
      if (receivedData.indexOf(",") != -1) {
        String latStr = receivedData.substring(0, receivedData.indexOf(","));
        String lngStr = receivedData.substring(receivedData.indexOf(",") + 1);

        // targetLat = latStr.toFloat();
        // targetLng = lngStr.toFloat();

        Serial.print("Received Latitude: ");
        Serial.println(targetLat, 6);
        Serial.print("Received Longitude: ");
        Serial.println(targetLng, 6);
      } else {
        Serial.println("Invalid data format. Expected 'latitude,longitude'.");
      }

      // Echo the received data back to the client
      webSocket.sendTXT(client_num, "Received: " + receivedData);
      break;
  }
}

void calculateBearing() {
  float deltaLng = radians(targetLng - currentLng);
  float currentLatRad = radians(currentLat);
  float targetLatRad = radians(targetLat);

  float y = sin(deltaLng) * cos(targetLatRad);
  float x = cos(currentLatRad) * sin(targetLatRad) - 
            sin(currentLatRad) * cos(targetLatRad) * cos(deltaLng);

  float bearingRad = atan2(y, x);  // Bearing in radians
  bearing = degrees(bearingRad);  // Convert to degrees

  // Normalize the bearing to 0-360 degrees
  if (bearing < 0) {
    bearing += 360;
  }

  // Print the calculated bearing to Serial Monitor
  Serial.print("Bearing to target: ");
  Serial.println(bearing);

  // Calculate the distance (Haversine formula)
  float R = 6371;  // Radius of Earth in kilometers
  float dLat = radians(targetLat - currentLat);
  float dLng = radians(targetLng - currentLng);
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(radians(currentLat)) * cos(radians(targetLat)) * sin(dLng / 2) * sin(dLng / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  distance = R * c * 1000;  // Distance in meters
  Serial.print("Distance to target: ");
  Serial.print(distance);
  Serial.println(" meters");
}

void readMagnetometer() { int16_t rawX, rawY, rawZ;
  
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(DataRegisterBegin);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_Address, 6);

  if (Wire.available() >= 6) {
    rawX = Wire.read() << 8 | Wire.read();
    rawZ = Wire.read() << 8 | Wire.read();
    rawY = Wire.read() << 8 | Wire.read();

    // Calculate heading (compass)
    float heading = atan2(rawY, rawX);
    currentHeading = degrees(heading);

    // Normalize the heading to 0-360 degrees
    if (currentHeading < 0) {
      currentHeading += 360;
    }

    // Print the current heading from the compass
    Serial.print("Current Heading: ");
    Serial.println(currentHeading);
  }
}

void adjustHeading() {
  // Check if current heading is greater or lesser than the target bearing and rotate accordingly
  if (currentHeading < bearing) {
    turnLeft();
  } else if (currentHeading > bearing) {
    turnRight();
  }
}

void turnLeft() {
  Serial.println("left");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  Serial.println("right");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveForward() {
  Serial.println("forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  Serial.println("stopped");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
