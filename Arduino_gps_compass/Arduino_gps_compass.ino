#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <math.h>

// Motor control pins
#define IN1 2
#define IN2 8
#define IN3 5
#define IN4 6

// Speed control
#define EN1 9
#define EN2 10
#define k 7
int speed = 170;
int a=0,b=0,g=0,f=0;

// Create TinyGPS++ object
TinyGPSPlus gps;

// Define RX and TX pins for SoftwareSerial
SoftwareSerial gpsSerial(4, 3); // RX, TX

// Magnetometer configuration
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Current GPS coordinates
float currentLat = 0.0;
float currentLng = 0.0;
float targetLat[k] = {23.8296169, 23.829437, 23.829472, 23.829671, 23.829472, 23.829635, 23.829462};
float targetLng[k] = {90.5672889, 90.567002, 90.567096, 90.567096, 90.567259, 90.567265, 90.567005};

// Distance and bearing
float distance = 5.0;
float bearing = 0.0;
float currentHeading = 0.0; // Heading from the magnetometer

void setup() {
  // Initialize serial communication
  // Serial.begin(9600);
  gpsSerial.begin(9600); // GPS communication

  pinMode(13, OUTPUT);

  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);

  // Magnetometer setup
  if (!mag.begin()) {
    // Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  // Serial.println("Waiting for GPS signal...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    // Parse GPS data
    if (gps.encode(c)) {
      // Serial.println("GPS Data Received");  
      displayGPSInfo();
    }
  }

  //Additional checks for GPS signal status
  if (!gps.location.isUpdated()) {
    // Serial.println("Waiting for GPS data...");
  }
  // Read magnetometer data
  
  readMagnetometer();

  // Calculate bearing and distance
  calculateBearing();

  // Adjust movement based on heading and distance
  if (abs(bearing - currentHeading) < 10) {
    moveForward(speed);
    digitalWrite(13, HIGH);
    g = 0;
  } else if (distance < 5) {
    stopMotors();
    delay(5000);
    if (f<k-1) {
    f++;}
  } else {
    adjustHeading();
    g = 1;
  }

  delay(100);
  // if(g==0) digitalWrite(13, HIGH);
  if (a&&b&&g)
  {
    digitalWrite(13, HIGH);
    delay(100); // Wait for 1 second
    // Turn the LED off
    digitalWrite(13, LOW);
  }
  a=0;b=0;
}

void displayGPSInfo() {

  a=1;
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLng = gps.location.lng();
    // Serial.print("Latitude: ");
    // Serial.print(currentLat, 6);
    // Serial.print(", Longitude: ");
    // Serial.println(currentLng, 6);
  } else {
    // Serial.println("Waiting for valid GPS signal...");
  }
}

void calculateBearing() {
  float deltaLng = radians(targetLng[f] - currentLng);
  float currentLatRad = radians(currentLat);
  float targetLatRad = radians(targetLat[f]);

  float y = sin(deltaLng) * cos(targetLatRad);
  float x = cos(currentLatRad) * sin(targetLatRad) -
            sin(currentLatRad) * cos(targetLatRad) * cos(deltaLng);

  bearing = degrees(atan2(y, x));
  if (bearing < 0) bearing += 360;

  // Calculate distance using Haversine formula
  float R = 6371.0; // Earth's radius in kilometers
  float dLat = radians(targetLat[f] - currentLat);
  float dLng = radians(targetLng[f] - currentLng);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(radians(currentLat)) * cos(radians(targetLat[f])) * sin(dLng / 2) * sin(dLng / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  distance = R * c * 1000; // Distance in meters

  // Serial.print("Bearing: ");
  // Serial.println(bearing);
  // Serial.print("Distance: ");
  // Serial.print(distance);
  // Serial.println(" meters");
}

void readMagnetometer() {
  sensors_event_t event;
  mag.getEvent(&event);
  b=1;
  // Serial.print("X: ");
  // Serial.print(event.magnetic.x);
  // Serial.print("  Y: ");
  // Serial.print(event.magnetic.y);
  // Serial.print("  Z: ");
  // Serial.print(event.magnetic.z);
  // Serial.println(" uT");

  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.22; // Adjust based on your location
  heading += declinationAngle;

  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI) heading -= 2 * PI;

  currentHeading = heading * 180 / M_PI;

  // Serial.print("Heading (degrees): ");
  // Serial.println(currentHeading);
}


void adjustHeading() {
  if (currentHeading < bearing) {
    if(abs(currentHeading - bearing) > 90){
      turnLeft(speed);
    }
    else {driftLeft();}
  } 
  if (currentHeading > bearing) {
    if(abs(currentHeading - bearing) > 90){
      turnRight(speed);
  }
    else {driftRight();}
}
}

void turnLeft(int speed) {
  // Serial.println("Turning left");
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void driftLeft() {
  // Serial.println("Turning left");
  analogWrite(EN1, 150);
  analogWrite(EN2, 70);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight(int speed) {
  // Serial.println("Turning right");
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void driftRight() {
  // Serial.println("Turning left");
  analogWrite(EN1, 70);
  analogWrite(EN2, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveForward(int speed) {
  // Serial.println("Moving forward");
  analogWrite(EN1, speed);
  analogWrite(EN2, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopMotors() {
  // Serial.println("Stopping motors");
  analogWrite(EN1, 0);
  analogWrite(EN2, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}