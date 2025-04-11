#include <MAX6675.h>
#include <SoftwareSerial.h>

#define MAX6675_CS 10
#define MQ2_PIN A0
#define DE 3
#define RE 4

MAX6675 thermocouple(MAX6675_CS);
SoftwareSerial mySerial(5, 2);  // For NPK sensor

void setup() {
  Serial.begin(9600);  // Initialize USB serial
  SPI.begin();
  pinMode(MQ2_PIN, INPUT);
  mySerial.begin(4800);
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
}

void loop() {
  // Read all sensors
  float temp = thermocouple.readTempC();
  int gas = analogRead(MQ2_PIN);
  
  // Read NPK sensor (simplified)
  uint8_t query[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
  uint8_t response[19];
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  mySerial.write(query, sizeof(query));
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  delay(1000);

  // Parse NPK data (with error handling)
  float soilH = -999.0, soilT = -999.0, soilPH = -999.0;
  int soilC = -999, N = -999, P = -999, K = -999;
  
  if (mySerial.available() >= 19) {
    mySerial.readBytes(response, 19);
    soilH = ((response[3] << 8) | response[4]) / 10.0;
    soilT = ((response[5] << 8) | response[6]) / 10.0;
    soilC = (response[7] << 8) | response[8];
    soilPH = ((response[9] << 8) | response[10]) / 10.0;
    N = (response[11] << 8) | response[12];
    P = (response[13] << 8) | response[14];
    K = (response[15] << 8) | response[16];
  }

  // Print as CSV (for easy parsing in Python)
  Serial.print(temp); Serial.print(",");
  Serial.print(gas); Serial.print(",");
  Serial.print(soilH); Serial.print(",");
  Serial.print(soilT); Serial.print(",");
  Serial.print(soilC); Serial.print(",");
  Serial.print(soilPH); Serial.print(",");
  Serial.print(N); Serial.print(",");
  Serial.print(P); Serial.print(",");
  Serial.println(K);

  delay(2000);  // Send data every 2 seconds
}