#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>


const char* ssid = "GUB";
const char* password = "GUB!@#2023";

// const char* ssid = "Hotspot";
// const char* password = "123456789";

// const char* ssid = "Room_506";
// const char* password = "greeN@121";

// const char* ssid = "Hepnox";
// const char* password = "Hepnox-Password";

WebSocketsServer webSocket = WebSocketsServer(81);

Servo chocolateServo;

const int chocolateServoPin = 18; // Update pin number as needed

int pos = 0;

void setup() {
  Serial.begin(115200); // ESP32 default baud rate is 115200
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  chocolateServo.attach(chocolateServoPin);
  chocolateServo.write(pos);
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");
}

void loop() {
  webSocket.loop();
  // Serial.println(chocolateServo.read());
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    if (msg.startsWith("choco:")) {
      delay(4000);
      pos += 90;
      Serial.print("Changing Pos: ");
      Serial.println(pos);
      moveServoGradually(chocolateServo, pos);
    if (chocolateServo.read() >= 178) {
      delay(2000);
      moveServoGradually(chocolateServo,0);
      pos = 0;
      }
    }
  }
}

void moveServoGradually(Servo& servo, int targetPos) {
  int currentPos = servo.read();
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos++) {
      servo.write(pos);
      delay(15);
    }
  } else {
    for (int pos = currentPos; pos >= targetPos; pos--) {
      servo.write(pos);
      delay(15);
    }
  }
}
