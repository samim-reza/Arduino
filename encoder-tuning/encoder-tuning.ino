
const int motorRPWM = D1;  // Right PWM pin
const int motorLPWM = D2;  // Left PWM pin
const int motorREN = D3;   // Right enable pin
const int motorLEN = D4;   // Left enable pin

const int encoderPinA = D5;
const int encoderPinB = D6;

volatile int pulseCount = 0;
const int pulsesPerCycle = 320;  // Number of pulses before stopping
bool motorRunning = false;

// Function to count encoder pulses
void IRAM_ATTR encoderISR() {
  pulseCount++;
}

void setup() {
  // Set motor control pins as outputs
  pinMode(motorRPWM, OUTPUT);
  pinMode(motorLPWM, OUTPUT);
  pinMode(motorREN, OUTPUT);
  pinMode(motorLEN, OUTPUT);

  // Set encoder pins
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);

  // Attach interrupt to encoder pin A
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);

  // Enable both sides of the motor driver
  digitalWrite(motorREN, HIGH);  // Enable right motor side
  digitalWrite(motorLEN, HIGH);  // Enable left motor side

  Serial.begin(9600);
}

void loop() {
  if (!motorRunning) {
    // Reset pulse counter
    pulseCount = 0;

    // Start motor (forward direction)
    analogWrite(motorRPWM, 255);  // Full speed for the right side
    analogWrite(motorLPWM, 0);    // No speed for the left side

    motorRunning = true;
    Serial.println("Motor started");
  }
  Serial.println(pulseCount);
  // Check if the motor has reached the desired number of pulses
  if (pulseCount >= pulsesPerCycle) {
    // Stop the motor
    analogWrite(motorRPWM, 0);  // Stop the motor
    analogWrite(motorLPWM, 0);  // Stop the motor
    Serial.println("Motor stopped after 12 pulses");

    // Delay for 3 seconds
    delay(3000);

    // Reset motorRunning flag to start the next cycle
    motorRunning = false;
  }
}
