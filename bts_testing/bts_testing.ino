const int actuator1A = 4;
const int actuator1B = 5;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);

  pinMode(actuator1A, OUTPUT);
  pinMode(actuator1B, OUTPUT);
}

void loop() {
  // Activate actuator (HIGH for 3 seconds)
  digitalWrite(actuator1A, HIGH);
  digitalWrite(actuator1B, LOW);
  Serial.println("Actuator ON");
  delay(3000);
  
  // Deactivate actuator (LOW for 3 seconds)
  digitalWrite(actuator1A, LOW);
  digitalWrite(actuator1B, HIGH);
  Serial.println("Actuator OFF");
  delay(3000);
}