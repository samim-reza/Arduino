#include <Wire.h>
#include <math.h>

#define HMC5883L_Address 0x1E
#define DataRegisterBegin 0x03  // Start of data registers for X, Z, and Y axes

void setup() {
  // Start the serial communication for debugging
  Serial.begin(9600);
  
  // Start the I2C communication (using default Arduino Uno pins for SDA and SCL)
  Wire.begin();  // Uses A4 for SDA and A5 for SCL on Arduino Uno
  
  // Configure the HMC5883L to continuous measurement mode
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(0x02);  // Mode register
  Wire.write(0x00);  // Set to continuous measurement mode
  Wire.endTransmission();
  
  delay(100);  // Wait for the sensor to initialize
}

void loop() {
  // Request 6 bytes of data from the sensor (2 bytes each for X, Z, and Y axes)
  Wire.beginTransmission(HMC5883L_Address);
  Wire.write(DataRegisterBegin);  // Start reading from the data register
  Wire.endTransmission();
  
  Wire.requestFrom(HMC5883L_Address, 6);  // Request 6 bytes (2 bytes for each axis)

  if (Wire.available() == 6) {
    // Read the 6 bytes of data
    int16_t x = (Wire.read() << 8) | Wire.read();  // Combine high and low byte for X-axis
    int16_t z = (Wire.read() << 8) | Wire.read();  // Combine high and low byte for Z-axis
    int16_t y = (Wire.read() << 8) | Wire.read();  // Combine high and low byte for Y-axis

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
    Serial.print("current: ");
    Serial.print(headingDegrees);
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

    // Print the cardinal direction
    Serial.print("\nHeading: ");
    Serial.println(direction);
  }

  delay(500);  // Delay to make output readable
}
