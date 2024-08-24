#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>


const char* ssid = "Hepnox";
const char* password = "Hepnox-Password";


AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {

    Serial.printf("Received data: %s\n", data);

    client->text("Message received");
  }
}

void setup() {

  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  // Serve static files from SPIFFS (if you have frontend files to serve)
  // if(!SPIFFS.begin()){
  //   Serial.println("An Error has occurred while mounting SPIFFS");
  //   return;
  // }
  // server.serveStatic("/", SPIFFS, "/");

  server.begin();
}

void loop() {
  ws.cleanupClients();
  int i = 0;
  ws.textAll(String(i++));
}
