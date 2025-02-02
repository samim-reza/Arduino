#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "index_html.h"
#include "styles_css.h"
#include "script_js.h"

const char* ssid = "ESP8266";
const char* password = "12345678";

ESP8266WebServer server(80);

bool ledState = false;

void handleRoot() {
  String html = String(INDEX_HTML);
  html.replace("/* Styles will be inserted here */", STYLES_CSS);
  html.replace("// JavaScript will be inserted here", SCRIPT_JS);
  server.send(200, "text/html", html);
}

void handleToggle() {
  ledState = !ledState;
  digitalWrite(LED_BUILTIN, ledState ? LOW : HIGH);  // LOW turns the LED on for most ESP8266 boards
  server.send(200, "text/plain", ledState ? "LED Status: ON" : "LED Status: OFF");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn LED off initially

  // Set up Access Point
  WiFi.softAP(ssid, password);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}