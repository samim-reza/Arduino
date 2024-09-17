#include <ros.h>
#include <std_msgs/String.h>

// Define motor pins
int motor1Pin1 = 9;  // Motor 1 PWM pin
int motor1Pin2 = 10; // Motor 1 direction pin
int motor2Pin1 = 5;  // Motor 2 PWM pin
int motor2Pin2 = 6;  // Motor 2 direction pin

int motor_speed = 20;

ros::NodeHandle nh;

// Function to control the motors based on the received message
void controlCallback(const std_msgs::String &msg) {
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
    analogWrite(motor1Pin1, motor_speed);
    analogWrite(motor2Pin1, motor_speed);
    analogWrite(motor1Pin2, 0);
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
    analogWrite(motor1Pin1, 0);
    analogWrite(motor2Pin1, 0);
    analogWrite(motor1Pin2, motor_speed);
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


// Create a subscriber to the topic 'soccer_rpm'
ros::Subscriber<std_msgs::String> sub("soccer_rpm", controlCallback);

void setup() {
  // Initialize motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Start ROS node handle
  nh.initNode();

  // Subscribe to the 'soccer_rpm' topic
  nh.subscribe(sub);
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();
  delay(10);
}
