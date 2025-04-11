#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(2, 3); // RX, TX

#define SFE_UBLOX_DISABLE_NMEA
#define SFE_UBLOX_DISABLE_I2C
#define SFE_UBLOX_DISABLE_LOGGING
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
SFE_UBLOX_GNSS myGNSS;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(38400);

  if (myGNSS.begin(gpsSerial)) {
    Serial.println("GPS connected.");
    myGNSS.setUART1Output(COM_TYPE_UBX);
    myGNSS.saveConfiguration();
  } else {
    Serial.println("GPS not detected.");
  }
}

void loop() {
  if (myGNSS.getPVT()) {
    Serial.print("Fix type: ");
    Serial.println(myGNSS.getFixType()); // 0 = No fix

    Serial.print("Satellites used: ");
    Serial.println(myGNSS.getSIV()); // SIV = Satellites in View

    Serial.print("Year: ");
    Serial.print(myGNSS.getYear());
    Serial.print(" Month: ");
    Serial.print(myGNSS.getMonth());
    Serial.print(" Day: ");
    Serial.println(myGNSS.getDay());

    Serial.print("Time: ");
    Serial.print(myGNSS.getHour());
    Serial.print(":");
    Serial.print(myGNSS.getMinute());
    Serial.print(":");
    Serial.println(myGNSS.getSecond());

    Serial.println("------");
  }
  delay(1000);
}
