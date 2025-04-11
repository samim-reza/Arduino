#include <Adafruit_GPS.h>
#include <HardwareSerial.h>

// Configure hardware serial (adjust pins per your ESP32 board)
#define GPS_RX 16  // GPIO16 = RX2
#define GPS_TX 17  // GPIO17 = TX2
HardwareSerial gpsSerial(2);  // Using UART2

Adafruit_GPS GPS(&gpsSerial);

// WiFi credentials (optional for NTRIP)
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // Initialize GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);  // Higher refresh rate
  GPS.sendCommand(PGCMD_ANTENNA);

  // For WiFi/NTRIP (optional)
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) delay(500);
  // Serial.println("WiFi connected");
}

void loop() {
  // Read GPS data
  while (gpsSerial.available()) {
    char c = GPS.read();
    if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA())) {
      printGPSData();
    }
  }
}

void printGPSData() {
  if (GPS.fix) {
    Serial.print("\nFix: ");
    if (GPS.fixquality == 4) Serial.print("RTK Fixed");
    else if (GPS.fixquality == 5) Serial.print("RTK Float");
    else Serial.print("Quality: " + String(GPS.fixquality));

    Serial.printf("\nLat: %.8f", GPS.latitudeDegrees);
    Serial.printf("\nLon: %.8f", GPS.longitudeDegrees);
    Serial.printf("\nAlt: %.2fm", GPS.altitude);
    Serial.printf("\nSatellites: %d\n", (int)GPS.satellites);
  }
}

// Add NTRIP client functions here for RTCM corrections