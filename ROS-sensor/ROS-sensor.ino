#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
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

long duration;
int distance;

ros::NodeHandle nh;

std_msgs::Float32 distance_msg;
std_msgs::Float32 temperature_msg;
std_msgs::Float32 humidity_msg;
sensor_msgs::Imu imu_msg;

ros::Publisher distance_pub("distance", &distance_msg);
ros::Publisher temperature_pub("temperature", &temperature_msg);
ros::Publisher humidity_pub("humidity", &humidity_msg);
ros::Publisher imu_pub("imu", &imu_msg);

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(9600);

  if (!accel.begin()) {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
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
  
  nh.getHardware()->setConnection(nh.getHardware()->mySerialHardware); // Use hardware serial
  nh.initNode();
  
  nh.advertise(distance_pub);
  nh.advertise(temperature_pub);
  nh.advertise(humidity_pub);
  nh.advertise(imu_pub);
}

void loop() {
  nh.spinOnce();
  
  // Publish distance
  distance_msg.data = measureDistance();
  distance_pub.publish(&distance_msg);
  
  // Publish temperature and humidity
  publishTemperatureHumidity();

  // Publish IMU data
  publishIMU();
  
  delay(1000);
}

float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);
  return distance;
}

void publishTemperatureHumidity() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  temperature_msg.data = t;
  humidity_msg.data = h;

  temperature_pub.publish(&temperature_msg);
  humidity_pub.publish(&humidity_msg);

  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.print(" Humidity: ");
  Serial.println(h);
}

void publishIMU() {
  sensors_event_t event; 
  accel.getEvent(&event);

  imu_msg.linear_acceleration.x = event.acceleration.x;
  imu_msg.linear_acceleration.y = event.acceleration.y;
  imu_msg.linear_acceleration.z = event.acceleration.z;

  imu_pub.publish(&imu_msg);

  Serial.print("IMU ax: ");
  Serial.print(event.acceleration.x);
  Serial.print(" ay: ");
  Serial.print(event.acceleration.y);
  Serial.print(" az: ");
  Serial.println(event.acceleration.z);
}
