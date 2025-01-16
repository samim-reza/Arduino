#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Create TinyGPS++ object
TinyGPSPlus gps;

// Define RX and TX pins for SoftwareSerial
SoftwareSerial gpsSerial(4, 3); // RX, TX

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  // Initialize SoftwareSerial for GPS
  gpsSerial.begin(9600);

  Serial.println("Waiting for GPS signal...");
}

void loop() {
  // Read data from GPS module
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    // Parse GPS data
    if (gps.encode(c)) {
      displayGPSInfo();
    }
  }
}

void displayGPSInfo() {
  // Check if location data is valid
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Waiting for valid GPS signal...");
  }
}
