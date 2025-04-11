#include <SoftwareSerial.h>

#define DE 3
#define RE 4

// Create SoftwareSerial instance for communication with RS485 sensor
SoftwareSerial mySerial(5, 2);  // RX, TX

void setup() {
  Serial.begin(9600);  // Initialize Serial Monitor
  mySerial.begin(4800); // Baud rate for RS485 communication
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
}

void loop() {
  uint8_t queryData[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08 };
  uint8_t receivedData[19];

  // Enable Transmit Mode (RS485)
  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);

  // Send Modbus request to the sensor
  mySerial.write(queryData, sizeof(queryData));

  // Disable Transmit Mode (switch to Receive mode)
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);

  delay(1000);

  if (mySerial.available() >= sizeof(receivedData)) {
    // Read the response
    mySerial.readBytes(receivedData, sizeof(receivedData));

    // Parse received data
    uint16_t soilHumidity = (receivedData[3] << 8) | receivedData[4];
    uint16_t soilTemperature = (receivedData[5] << 8) | receivedData[6];
    uint16_t soilConductivity = (receivedData[7] << 8) | receivedData[8];
    uint16_t soilPH = (receivedData[9] << 8) | receivedData[10];
    uint16_t nitrogen = (receivedData[11] << 8) | receivedData[12];
    uint16_t phosphorus = (receivedData[13] << 8) | receivedData[14];
    uint16_t potassium = (receivedData[15] << 8) | receivedData[16];

    // Print the parsed values
    Serial.print("Soil Humidity: ");
    Serial.println((float)soilHumidity / 10.0);
    Serial.print("Soil Temperature: ");
    Serial.println((float)soilTemperature / 10.0);
    Serial.print("Soil Conductivity: ");
    Serial.println(soilConductivity);
    Serial.print("Soil pH: ");
    Serial.println((float)soilPH / 10.0);
    Serial.print("Nitrogen: ");
    Serial.println(nitrogen);
    Serial.print("Phosphorus: ");
    Serial.println(phosphorus);
    Serial.print("Potassium: ");
    Serial.println(potassium);
    Serial.println("\n\n");
  }
  
  delay(2500);  // Delay before the next query
}