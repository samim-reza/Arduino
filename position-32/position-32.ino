#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Create instances for TinyGPS++ and Serial communication
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// Current GPS coordinates
float currentLat = 0.0;
float currentLng = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for serial port to initialize
  Serial.println("GPS Test Started!");
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
}


void loop() {
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    if (gps.encode(c)) {
      if (gps.location.isUpdated()) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();

        // Print with full precision
        Serial.print("Latitude: ");
        Serial.println(currentLat, 6); // 6 digits after the decimal point
        Serial.print("Longitude: ");
        Serial.println(currentLng, 6); // 6 digits after the decimal point
      }
    }
  }
  delay(1000);  // Add a delay to avoid flooding the serial monitor
}
