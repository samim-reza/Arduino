#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define trigPin D1
#define echoPin D5
#define tempPin A0
#define humidityPin 0 // GPIO 0 corresponds to A1 on ESP8266

const char* ssid = "GUB";
const char* password = "GUB!@#2023";

long duration;
int distance;
int humidity; // Variable to store humidity value
bool ledState = false;

ESP8266WebServer server(80);

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/distance", handleDistance);
  server.on("/toggle", toggle);
  server.on("/temperature", handleTemperature);
  server.on("/humidity", handleHumidity); // New route for humidity
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}

void handleDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  String response = "{ \"distance\": " + String(distance) + " }";
  Serial.print("Distance: ");
  Serial.println(distance);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);
}

void toggle() {
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);

  String response = "{ \"led\": \"" + String(ledState ? "on" : "off") + "\" }";
  Serial.print("LED: ");
  Serial.println(ledState ? "ON" : "OFF");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);
}

void handleTemperature() {
  int sensorValue = analogRead(tempPin);
  float voltage = sensorValue * (3.3 / 1023.0);
  float temperatureC = voltage * 100.0;

  String response = "{ \"temperature\": " + String(temperatureC) + " }";
  Serial.print("Temperature: ");
  Serial.println(temperatureC);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);
}

void handleHumidity() {
  int sensorValue = analogRead(humidityPin);
  humidity = sensorValue * (100.0 / 1023.0);

  String response = "{ \"humidity\": " + String(humidity) + " }";
  Serial.print("Humidity: ");
  Serial.println(humidity);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);
}
