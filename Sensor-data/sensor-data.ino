#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define trigPin D4
#define echoPin D5
#define DHTPIN D7
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);


 const char* ssid = "GUB";
const char* password = "GUB!@#2023";

// const char* ssid = "Room_506";
// const char* password = "greeN@121";

  // const char* ssid = "Hepnox";
  // const char* password = "Hepnox-Password";

long duration;
int distance;

ESP8266WebServer server(80);

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);

  if(!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  dht.begin();

  server.on("/distance", handleDistance);
  server.on("/temperature", handleTemperature);
  server.on("/imu", handleIMU);
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
  // delay(2000);
}

void handleTemperature() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    server.send(500, "text/plain", "Failed to read from DHT sensor!");
    return;
  }

  String response = "{ \"temperature\": " + String(t) + " , \"humidity\": " + String(h) + " }";
  Serial.println(response);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);
  // delay(2000);
}

void handleIMU() {
  sensors_event_t event; 
  accel.getEvent(&event);

  float ax = event.acceleration.x;
  float ay = event.acceleration.y;
  float az = event.acceleration.z;

  String response = "{";
  response += "\"ax\": " + String(ax) + ",";
  response += "\"ay\": " + String(ay) + ",";
  response += "\"az\": " + String(az);
  response += "}";

  Serial.println(response);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);
  //  delay(2000);
}