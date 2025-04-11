#include <SPI.h>
#include <MAX6675.h>
#include <SoftwareSerial.h>

// Pin Definitions
#define MQ2_PIN A0
#define DE 3
#define RE 4
#define MAX6675_CS 10

// RS485 SoftwareSerial setup
SoftwareSerial mySerial(5, 2);  // RX, TX

// MAX6675 Thermocouple sensor
MAX6675 thermocouple(MAX6675_CS);

// Timing variables
unsigned long lastMQ2Time = 0;
unsigned long lastMAX6675Time = 0;
unsigned long lastRS485SendTime = 0;
bool waitingForRS485Response = false;

// Interval constants
const unsigned long MQ2_INTERVAL = 1000;
const unsigned long MAX6675_INTERVAL = 1000;
const unsigned long RS485_INTERVAL = 3500;

void setup() {
  Serial.begin(115200);
  
  // MQ-2 Sensor setup
  pinMode(MQ2_PIN, INPUT);
  
  // RS485 setup
  mySerial.begin(4800);
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
  
  // MAX6675 setup
  SPI.begin();
  Serial.println("All sensors initialized.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Read MQ-2 Gas Sensor
  if (currentMillis - lastMQ2Time >= MQ2_INTERVAL) {
    int sensorValue = analogRead(MQ2_PIN);
    Serial.print("MQ-2 Value: ");
    Serial.println(sensorValue);
    lastMQ2Time = currentMillis;
  }

  // Read MAX6675 Temperature Sensor
  if (currentMillis - lastMAX6675Time >= MAX6675_INTERVAL) {
    float temperature = thermocouple.readTempC();
    if (isnan(temperature)) {
      Serial.println("Error reading MAX6675");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
    }
    lastMAX6675Time = currentMillis;
  }

  // Handle RS485 Soil Sensor
  if (!waitingForRS485Response && (currentMillis - lastRS485SendTime >= RS485_INTERVAL)) {
    // Send Modbus query
    uint8_t queryData[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
    
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    mySerial.write(queryData, sizeof(queryData));
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    
    waitingForRS485Response = true;
    lastRS485SendTime = currentMillis;
  }

  // Process RS485 response
  if (waitingForRS485Response && (currentMillis - lastRS485SendTime >= 1000)) {
    uint8_t receivedData[19];
    
    if (mySerial.available() >= sizeof(receivedData)) {
      mySerial.readBytes(receivedData, sizeof(receivedData));
      
      // Parse sensor data
      uint16_t soilHumidity = (receivedData[3] << 8) | receivedData[4];
      uint16_t soilTemperature = (receivedData[5] << 8) | receivedData[6];
      uint16_t soilConductivity = (receivedData[7] << 8) | receivedData[8];
      uint16_t soilPH = (receivedData[9] << 8) | receivedData[10];
      uint16_t nitrogen = (receivedData[11] << 8) | receivedData[12];
      uint16_t phosphorus = (receivedData[13] << 8) | receivedData[14];
      uint16_t potassium = (receivedData[15] << 8) | receivedData[16];

      // Print parsed values
      Serial.print("Soil Humidity: ");
      Serial.println(soilHumidity / 10.0);
      Serial.print("Soil Temperature: ");
      Serial.println(soilTemperature / 10.0);
      Serial.print("Soil Conductivity: ");
      Serial.println(soilConductivity);
      Serial.print("Soil pH: ");
      Serial.println(soilPH / 10.0);
      Serial.print("Nitrogen: ");
      Serial.println(nitrogen);
      Serial.print("Phosphorus: ");
      Serial.println(phosphorus);
      Serial.print("Potassium: ");
      Serial.println(potassium);
      Serial.println("-------------------");
    } else {
      Serial.println("RS485 Error: Incomplete data");
    }
    waitingForRS485Response = false;
  }
}