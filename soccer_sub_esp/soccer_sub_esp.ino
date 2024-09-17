#include <ros.h>
#include <std_msgs/String.h>
#include <ESP8266WiFi.h>


int motor1Pin1 = D1;
int motor1Pin2 = D2;
int motor2Pin1 = D5;
int motor2Pin2 = D6;

int motor_speed = 20;

IPAddress server(192, 168, 0, 188); // Replace with your ROS master IP
WiFiClient client;

// WiFi credentials
// const char* ssid = "Hepnox"; 
// const char* password = "Hepnox-Password";
const char* ssid = "Room_506";
const char* password = "greeN@121";

// const char* ssid = "GUB";
// const char* password = "GUB!@#2023";

// Forward declaration of the controlCallback function
void controlCallback(const std_msgs::String &msg);

// -- WiFiHardware CLASS for ROS Communication over WiFi --

class WiFiHardware {
  public:
const char* password = "greeN@121";
    WiFiHardware() {}

    void init() {
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      Serial.println("WiFi connected");
      Serial.println(WiFi.localIP());
      client.connect(server, 11411); // Port 11411 for ROS communication
      if (client.connected()) {
        Serial.println("Connected to ROS master.");
      } else {
        Serial.println("Failed to connect to ROS master.");
      }

    }



    int read() {
      return client.read();  // Read from the TCP connection
    }

    void write(uint8_t* data, int length) {
      for (int i = 0; i < length; i++) {
        client.write(data[i]);  // Write to the TCP connection
      }
    }

    unsigned long time() {
      return millis(); // Return the system time
    }
};

// ROS node handle and publisher
ros::NodeHandle_<WiFiHardware> nh;
ros::Subscriber<std_msgs::String> sub("soccer_rpm", controlCallback);

// Function to control the motors based on the received message
void controlCallback(const std_msgs::String &msg) {

    Serial.print("Received command: ");
 

  char command = msg.data[0];

  if (command == 'w') {
    // Move forward
    analogWrite(motor1Pin1, motor_speed);
    analogWrite(motor2Pin1, motor_speed);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin2, 0);
    nh.loginfo("Forward");
  } else if (command == 'a') {
    // Turn left
    analogWrite(motor1Pin1, 0);
    analogWrite(motor2Pin1,motor_speed);
    analogWrite(motor1Pin2, motor_speed);
    analogWrite(motor2Pin2, 0);
    nh.loginfo("Left");
  } else if (command == 's') {
    // Move backward
    analogWrite(motor1Pin1, 0);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor1Pin2, motor_speed);
    analogWrite(motor2Pin2, motor_speed);
    nh.loginfo("Backward");
  } else if (command == 'd') {
    // Turn right
    analogWrite(motor1Pin1, motor_speed);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin2, motor_speed);
    nh.loginfo("Right");
  } else if (command == 'q') {
    // Increase speed by 10
    motor_speed += 10;
    char log_msg[50];
    sprintf(log_msg, "Speed increased to %d", motor_speed);
    nh.loginfo(log_msg);
  } else if (command == 'e') {
    // Decrease speed by 10
    motor_speed -= 10;
    char log_msg[50];
    sprintf(log_msg, "Speed decreased to %d", motor_speed);
    nh.loginfo(log_msg);
  } else if (command == 'n') {
    // Stop
    analogWrite(motor1Pin1, 0);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor1Pin2, 0);
    analogWrite(motor2Pin2, 0);
    nh.loginfo("Stop");
  }
}

// -- SETUP --

void setup() {
  Serial.begin(115200);  // Start serial communication for debugging

  // Initialize motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Initialize ROS node over WiFi
  Serial.println("Initializing ROS...");
  nh.initNode();
  nh.subscribe(sub);
  Serial.println("ROS Initialized");
}

// -- LOOP --

void loop() {
  nh.spinOnce();  // Handle ROS communication
  delay(10);      // Small delay for stability
}
