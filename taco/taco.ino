#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "Room_506";
const char* password = "greeN@121";

ESP8266WebServer server(80);

volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseDuration = 0;
volatile bool pulseReceived = false;

float rpm = 0;

int irSensorPin = D2;
unsigned long debounceDelay = 2000; // 2ms debounce to avoid noise
unsigned long rpmTimeout = 1000000; // 1 second timeout for no pulses

void IRAM_ATTR handlePulse() {
  unsigned long currentPulseTime = micros(); // Time in microseconds
  
  // Debounce to ignore noise
  if (currentPulseTime - lastPulseTime > debounceDelay) {
    pulseDuration = currentPulseTime - lastPulseTime;
    lastPulseTime = currentPulseTime;
    pulseReceived = true;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(irSensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(irSensorPin), handlePulse, FALLING);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP()); 

  server.on("/", handleWebRequest);
  server.begin();
  Serial.println("Server started");
}

void loop() {
  server.handleClient();
  
  // If a pulse has been received, calculate the RPM
  if (pulseReceived) {
    rpm = 60.0 * 1000000.0 / pulseDuration; // Convert pulse duration to RPM
    pulseReceived = false;
  }
  
  // If no pulse received for more than 1 second, assume RPM is 0
  if (micros() - lastPulseTime > rpmTimeout) {
    rpm = 0;
  }

  Serial.print("RPM: ");
  Serial.println(rpm);
}

void handleWebRequest() {
  String rpmString = "{ \"rpm\": " + String(rpm) + " }";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "text/plain", rpmString);
}
