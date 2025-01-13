#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

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

void setup() {

  Serial.begin(115200);
  Serial.println();
  
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
  // Initialize serial communication
  while (!Serial);
  Serial.println("GPS and Magnetometer Test Started!");

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

  // Process Magnetometer data
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(DataRegisterBegin);
  Wire.endTransmission();

  Wire.requestFrom(HMC5883L_Address, 6);

  if (Wire.available() == 6) {
    int16_t x = (Wire.read() << 8) | Wire.read();
    int16_t z = (Wire.read() << 8) | Wire.read();
    int16_t y = (Wire.read() << 8) | Wire.read();

    // Print raw magnetometer data
    Serial.print("X: ");
    Serial.print(x);
    Serial.print("\tZ: ");
    Serial.print(z);
    Serial.print("\tY: ");
    Serial.println(y);

    // Calculate pitch and roll
    float pitch = atan2((float)-y, sqrt((float)(x * x) + (float)(z * z)));
    float roll = atan2((float)x, (float)z);

    // Correct the X and Y values for tilt
    float X_comp = x * cos(pitch) + z * sin(pitch);
    float Y_comp = y * cos(roll) + z * sin(roll);

    // Calculate heading (in radians)
    float heading = atan2(Y_comp, X_comp);

    // Normalize the heading to be between 0 and 360 degrees
    if (heading < 0) {
      heading += 2 * PI;
    }

    // Convert heading to degrees
    float headingDegrees = heading * 180.0 / PI;

    // Map the heading degrees to cardinal directions
    String direction = "Unknown";
    if (headingDegrees >= 337.5 || headingDegrees < 22.5) {
      direction = "North";
    } else if (headingDegrees >= 22.5 && headingDegrees < 67.5) {
      direction = "North-East";
    } else if (headingDegrees >= 67.5 && headingDegrees < 112.5) {
      direction = "East";
    } else if (headingDegrees >= 112.5 && headingDegrees < 157.5) {
      direction = "South-East";
    } else if (headingDegrees >= 157.5 && headingDegrees < 202.5) {
      direction = "South";
    } else if (headingDegrees >= 202.5 && headingDegrees < 247.5) {
      direction = "South-West";
    } else if (headingDegrees >= 247.5 && headingDegrees < 292.5) {
      direction = "West";
    } else if (headingDegrees >= 292.5 && headingDegrees < 337.5) {
      direction = "North-West";
    }

    // Print heading and direction
    Serial.print("Heading: ");
    Serial.print(headingDegrees, 2);
    Serial.print(" degrees (Direction: ");
    Serial.print(direction);
    Serial.println(")");
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
        String lat = receivedData.substring(0, receivedData.indexOf(","));
        String lng = receivedData.substring(receivedData.indexOf(",") + 1);

        Serial.print("Received Latitude: ");
        Serial.println(lat);
        Serial.print("Received Longitude: ");
        Serial.println(lng);
      } else {
        Serial.println("Invalid data format. Expected 'latitude,longitude'.");
      }

      // Echo the received data back to the client
      webSocket.sendTXT(client_num, "Received: " + receivedData);
      break;
  }
}

