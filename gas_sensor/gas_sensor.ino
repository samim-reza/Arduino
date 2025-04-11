#define MQ2_PIN A0  // Analog pin

void setup() {
    Serial.begin(115200);
    pinMode(MQ2_PIN, INPUT);
}

void loop() {
    int sensorValue = analogRead(MQ2_PIN);
    Serial.print("MQ-2 Sensor Value: ");
    Serial.println(sensorValue);
    delay(1000);  // Wait for 1 second
}