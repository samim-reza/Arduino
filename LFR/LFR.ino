#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <QTRSensors.h>

QTRSensors qtr;

// const char* ssid = "GUB";
// const char* password = "GUB!@#2023";

const char* ssid = "Room_506";
const char* password = "greeN@121";

ESP8266WebServer server(80);

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup()
{
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){1, 2, 3, 4, 5, 6, 7, 8}, SensorCount);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);

  server.on("/", read);
  server.begin();
  Serial.println("HTTP server started");
}

void loop()
{
  server.handleClient();
}

void read()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  String response = "{";
  response += "\"position\": " + String(position) + ",";
  response += "\"sensors\": [";
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    response += String(sensorValues[i]);
    if (i < SensorCount - 1)
    {
      response += ",";
    }
  }
  response += "]}";

  Serial.println("Sending JSON response:");
  Serial.println(response);

  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", response);

  delay(250);
}
