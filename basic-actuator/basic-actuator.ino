// Output variables to GPIO pins
const int in_A = 32; // Actuator UP
const int in_B = 33; // Actuator DOWN
const int in_C = 25; // Actuator UP
const int in_D = 26; // Actuator DOWN

void setup() {
  Serial.begin(115200);

  // Set GPIOs as outputs and turn them off initially
  pinMode(in_A, OUTPUT);
  digitalWrite(in_A, LOW);

  pinMode(in_B, OUTPUT);
  digitalWrite(in_B, LOW);

  pinMode(in_C, OUTPUT);
  digitalWrite(in_C, LOW);

  pinMode(in_D, OUTPUT);
  digitalWrite(in_D, LOW);

  Serial.println("Actuator control initialized.");
}

void loop() {
  // Move actuator UP for 3 seconds
  Serial.println("Actuator UP");
  digitalWrite(in_A, HIGH);
  digitalWrite(in_C, HIGH);
  delay(3000);
  digitalWrite(in_A, LOW);
  digitalWrite(in_C, LOW);

  // Move actuator DOWN for 3 seconds
  Serial.println("Actuator DOWN");
  digitalWrite(in_B, HIGH);
  digitalWrite(in_D, HIGH);
  delay(3000);
  digitalWrite(in_B, LOW);
  digitalWrite(in_D, LOW);
}


