volatile long encoderCount = 0;  // Stores encoder position
int lastAState = 0;

#define ENCODER_A 2  // Must be an interrupt pin
#define ENCODER_B 3  // Another interrupt pin

void setup() {
    Serial.begin(115200);

    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoder, CHANGE);
}

void loop() {
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount);
    delay(100);
}

void readEncoder() {
    int A = digitalRead(ENCODER_A);
    int B = digitalRead(ENCODER_B);

    if (A == B) {
        encoderCount++;  // Forward rotation
    } else {
        encoderCount--;  // Backward rotation
    }
}
