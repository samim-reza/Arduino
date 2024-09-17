const int motorA_RPWM = D1;  // Motor A Right PWM pin
const int motorA_LPWM = D2;  // Motor A Left PWM pin
const int motorA_EN = D3;    // Motor A enable pin

const int motorB_RPWM = D4;  // Motor B Right PWM pin
const int motorB_LPWM = D7;  // Motor B Left PWM pin
const int motorB_EN = D8;    // Motor B enable pin

const int encoderPinA1 = D5;  // Encoder pin A for motor A
const int encoderPinB1 = D6;  // Encoder pin B for motor A

const int encoderPinA2 = D9;  // Encoder pin A for motor B
const int encoderPinB2 = D10; // Encoder pin B for motor B

volatile int pulseCountA = 0;  // Pulse count for motor A
volatile int pulseCountB = 0;  // Pulse count for motor B
const int pulsesPerCycle = 320;  // Number of pulses before stopping
bool motorARunning = false;
bool motorBRunning = false;

// Function to count encoder pulses for motor A
void IRAM_ATTR encoderISRA() {
  pulseCountA++;
}

// Function to count encoder pulses for motor B
void IRAM_ATTR encoderISRB() {
  pulseCountB++;
}

void setup() {
  // Set motor control pins as outputs
  pinMode(motorA_RPWM, OUTPUT);
  pinMode(motorA_LPWM, OUTPUT);
  pinMode(motorA_EN, OUTPUT);

  pinMode(motorB_RPWM, OUTPUT);
  pinMode(motorB_LPWM, OUTPUT);
  pinMode(motorB_EN, OUTPUT);

  // Set encoder pins as inputs
  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);

  // Attach interrupts to encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), encoderISRA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), encoderISRB, RISING);

  // Enable both sides of the motor driver
  digitalWrite(motorA_EN, HIGH);  // Enable Motor A
  digitalWrite(motorB_EN, HIGH);  // Enable Motor B

  Serial.begin(9600);
}

void loop() {
  // Motor A control
  if (!motorARunning) {
    pulseCountA = 0;
    analogWrite(motorA_RPWM, 255);  // Full speed for Motor A
    analogWrite(motorA_LPWM, 0);    // No speed for the other direction
    motorARunning = true;
    Serial.println("Motor A started");
  }

  // Motor B control
  if (!motorBRunning) {
    pulseCountB = 0;
    analogWrite(motorB_RPWM, 255);  // Full speed for Motor B
    analogWrite(motorB_LPWM, 0);    // No speed for the other direction
    motorBRunning = true;
    Serial.println("Motor B started");
  }

  // Print pulse counts
  Serial.print("Motor A Pulse Count: ");
  Serial.println(pulseCountA);
  Serial.print("Motor B Pulse Count: ");
  Serial.println(pulseCountB);

  // Stop Motor A after the desired number of pulses
  if (pulseCountA >= pulsesPerCycle) {
    analogWrite(motorA_RPWM, 0);  // Stop Motor A
    analogWrite(motorA_LPWM, 0);
    Serial.println("Motor A stopped after pulses");
    motorARunning = false;
  }

  // Stop Motor B after the desired number of pulses
  if (pulseCountB >= pulsesPerCycle) {
    analogWrite(motorB_RPWM, 0);  // Stop Motor B
    analogWrite(motorB_LPWM, 0);
    Serial.println("Motor B stopped after pulses");
    motorBRunning = false;
  }

  // Optional delay to pause between loops
  delay(3000);
}
