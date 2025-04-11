#include <SPI.h>
#include <MAX6675.h>

// Define the Chip Select pin
#define MAX6675_CS   10   // Chip Select Pin (Pin 10 is typically used for SPI)

// Create an instance of the MAX6675 object (use the CS pin only)
MAX6675 thermocouple(MAX6675_CS);

void setup() {
  Serial.begin(115200);
  Serial.println("MAX6675 Temperature Sensor Initialized...");
  delay(500);  // Allow sensor to initialize
}

void loop() {
  // Read the temperature
  float temperature = thermocouple.readTempC();  // Use readTemperature() based on your library

  if (isnan(temperature)) {
    Serial.println("Error: Failed to read temperature");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
  }

  delay(1000); // Wait for 1 second before the next reading
}