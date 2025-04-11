void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    if (input == "PING") {
      Serial.println("PONG");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}