#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String msg;
ros::Publisher chatter("chatter", &msg);

void setup() {
  nh.initNode();
  nh.advertise(chatter);
}

void loop() {
  msg.data = "Hello from ESP32!";
  chatter.publish(&msg);
  nh.spinOnce();
  delay(1000);  // Delay for 1 second
}
